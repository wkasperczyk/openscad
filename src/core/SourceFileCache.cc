#include "core/SourceFileCache.h"
#include "core/StatCache.h"
#include "core/SourceFile.h"
#include "core/FileHasher.h"
#include "utils/printutils.h"
#include "openscad.h"
#include <ctime>
#include <boost/format.hpp>

#include <cstdio>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <algorithm>

/*!
   FIXME: Implement an LRU scheme to avoid having an ever-growing source file cache
 */

SourceFileCache *SourceFileCache::inst = nullptr;

/*!
   Reevaluate the given file and all its dependencies and recompile anything
   needing reevaluation. Updates the cache if necessary.
   The given filename must be absolute.

   Sets the given source file reference to the new file, or nullptr on any error (e.g. compile
   error or file not found).

   Returns the latest modification time of the file, its dependencies or includes.
 */
std::time_t SourceFileCache::evaluate(const std::string& mainFile, const std::string& filename, SourceFile *& sourceFile)
{
  sourceFile = nullptr;
  auto entry = this->entries.find(filename);
  bool found{entry != this->entries.end()};
  SourceFile *file{found ? entry->second.file : nullptr};

  // Don't try to recursively evaluate - if the file changes
  // during evaluation, that would be really bad.
  if (file && file->isHandlingDependencies()) return 0;

  // FIRST: Try hash-based cache lookup
  // We need to calculate the dependency chain hash for the file
  std::string dependencyChainHash;
  std::vector<std::string> dependencies;
  SourceFile *hashCachedFile = nullptr;
  
  // Calculate hash for current file and its dependencies
  dependencyChainHash = FileHasher::calculateFileHash(filename);
  if (!dependencyChainHash.empty()) {
    // For hash-based lookup, we need to read the file to extract its dependencies
    // This is a lightweight operation compared to full parsing
    std::string text;
    std::ifstream ifs(filename.c_str());
    if (ifs.is_open()) {
      text = STR(ifs.rdbuf(), "\n\x03\n", commandline_commands);
      ifs.close();
      
      // Create a temporary SourceFile to extract dependencies without full parsing
      // We'll use a simplified approach: if we have an already parsed version, use its dependencies
      if (file) {
        dependencies = file->getAllDependencies();
        dependencyChainHash = FileHasher::calculateDependencyChainHash(filename, dependencies);
        hashCachedFile = lookupByHash(dependencyChainHash);
      }
    }
  }
  
  // If hash cache hit, use it
  if (hashCachedFile) {
    sourceFile = hashCachedFile;
    PRINTDB("Using hash-cached library: %s (%p)", filename % hashCachedFile);
    // Still need to handle dependencies and return modification time
    std::time_t deps_mtime = hashCachedFile->handleDependencies(false);
    struct stat st;
    std::time_t file_mtime = (StatCache::stat(filename, st) == 0) ? st.st_mtime : 0;
    return std::max(deps_mtime, file_mtime);
  }

  // FALLBACK: Use existing time-based cache logic
  // Create cache ID
  struct stat st;
  bool valid = (StatCache::stat(filename, st) == 0);

  // If file isn't there, just return and let the cache retain the old file
  if (!valid) return 0;

  // If the file is present, we'll always cache some result
  std::string cache_id = str(boost::format("%x.%x") % st.st_mtime % st.st_size);

  cache_entry& cacheEntry = this->entries[filename];
  // Initialize entry, if new
  if (!found) {
    cacheEntry.file = nullptr;
    cacheEntry.parsed_file = nullptr;
    cacheEntry.cache_id = cache_id;
    cacheEntry.includes_mtime = st.st_mtime;
  }
  cacheEntry.mtime = st.st_mtime;

  bool shouldCompile = true;
  if (found) {
    // Files should only be recompiled if the cache ID changed
    if (cacheEntry.cache_id == cache_id) {
      shouldCompile = false;
      // Recompile if includes changed
      if (cacheEntry.parsed_file) {
        std::time_t mtime = cacheEntry.parsed_file->includesChanged();
        if (mtime > cacheEntry.includes_mtime) {
          cacheEntry.includes_mtime = mtime;
          shouldCompile = true;
        }
      }
    }
  }

#ifdef DEBUG
  // Causes too much debug output
  //if (!shouldCompile) LOG(message_group::NONE,,"Using cached library: %1$s (%2$p)",filename,file);
#endif

  // If cache lookup failed (non-existing or old timestamp), compile file
  if (shouldCompile) {
#ifdef DEBUG
    if (found) {
      PRINTDB("Recompiling cached library: %s (%s)", filename % cache_id);
    } else {
      PRINTDB("Compiling library '%s'.", filename);
    }
#endif

    std::string text;
    {
      std::ifstream ifs(filename.c_str());
      if (!ifs.is_open()) {
        LOG(message_group::Warning, "Can't open library file '%1$s'\n", filename);
        return 0;
      }
      text = STR(ifs.rdbuf(), "\n\x03\n", commandline_commands);
    }

    print_messages_push();

    delete cacheEntry.parsed_file;
    file = parse(cacheEntry.parsed_file, text, filename, mainFile, false) ? cacheEntry.parsed_file : nullptr;
    PRINTDB("compiled file: %s", filename);
    cacheEntry.file = file;
    cacheEntry.cache_id = cache_id;
    auto mod = file ? file : cacheEntry.parsed_file;
    if (!found && mod) cacheEntry.includes_mtime = mod->includesChanged();
    
    // Cache the result in hash-based cache if parsing succeeded
    if (file) {
      std::string newDependencyChainHash = file->calculateDependencyChainHash();
      if (!newDependencyChainHash.empty()) {
        std::vector<std::string> fileDependencies = file->getAllDependencies();
        cacheByHash(newDependencyChainHash, file, fileDependencies);
        PRINTDB("Cached compiled file by hash: %s -> %s", filename % newDependencyChainHash);
      }
    }
    
    print_messages_pop();
  }

  sourceFile = file;
  // FIXME: Do we need to handle include-only cases?
  std::time_t deps_mtime = file ? file->handleDependencies(false) : 0;

  return std::max({deps_mtime, cacheEntry.mtime, cacheEntry.includes_mtime});
}

void SourceFileCache::clear()
{
  this->entries.clear();
  this->hashEntries.clear();
}

SourceFile *SourceFileCache::lookup(const std::string& filename)
{
  auto it = this->entries.find(filename);
  return it != this->entries.end() ? it->second.file : nullptr;
}

void SourceFileCache::clear_markers() {
  for (const auto& entry : instance()->entries)
    if (auto lib = entry.second.file) lib->clearHandlingDependencies();
}

// Hash-based cache implementation
SourceFile *SourceFileCache::lookupByHash(const std::string& dependencyChainHash)
{
  auto it = this->hashEntries.find(dependencyChainHash);
  if (it != this->hashEntries.end()) {
    // Increment reference count and update last accessed time
    it->second.reference_count++;
    it->second.last_accessed = std::time(nullptr);
    return it->second.file;
  }
  return nullptr;
}

bool SourceFileCache::cacheByHash(const std::string& dependencyChainHash, SourceFile *sourceFile,
                                  const std::vector<std::string>& dependencies)
{
  if (!sourceFile || dependencyChainHash.empty()) {
    return false;
  }

  // Enforce memory limits before adding new entries
  enforceMemoryLimits();

  hash_cache_entry& entry = this->hashEntries[dependencyChainHash];
  entry.file = sourceFile;
  entry.dependencies = dependencies;
  entry.cache_time = std::time(nullptr);
  entry.last_accessed = std::time(nullptr);
  entry.reference_count = 1;

  return true;
}

std::string SourceFileCache::calculateDependencyChainHash(const std::string& filename,
                                                         const std::vector<std::string>& dependencies)
{
  return FileHasher::calculateDependencyChainHash(filename, dependencies);
}

void SourceFileCache::clearHashCache()
{
  this->hashEntries.clear();
}

// Memory management implementation
void SourceFileCache::releaseReference(const std::string& dependencyChainHash)
{
  auto it = this->hashEntries.find(dependencyChainHash);
  if (it != this->hashEntries.end() && it->second.reference_count > 0) {
    it->second.reference_count--;
    
    // Remove entry if no more references and it's old enough
    if (it->second.reference_count == 0) {
      std::time_t now = std::time(nullptr);
      // Keep entries for at least 60 seconds after last reference to allow reuse
      if (now - it->second.last_accessed > 60) {
        this->hashEntries.erase(it);
      }
    }
  }
}

void SourceFileCache::evictLRUEntries(size_t maxEntries)
{
  if (this->hashEntries.size() <= maxEntries) {
    return;
  }

  // Create vector of entries sorted by last accessed time (LRU first)
  std::vector<std::pair<std::time_t, std::string>> lruList;
  for (const auto& entry : this->hashEntries) {
    // Only consider entries with zero references for eviction
    if (entry.second.reference_count == 0) {
      lruList.push_back({entry.second.last_accessed, entry.first});
    }
  }

  // Sort by last accessed time (oldest first)
  std::sort(lruList.begin(), lruList.end());

  // Remove oldest entries until we're under the limit
  size_t entriesToRemove = this->hashEntries.size() - maxEntries;
  size_t removed = 0;
  
  for (const auto& lruEntry : lruList) {
    if (removed >= entriesToRemove) break;
    
    auto it = this->hashEntries.find(lruEntry.second);
    if (it != this->hashEntries.end() && it->second.reference_count == 0) {
      this->hashEntries.erase(it);
      removed++;
    }
  }
}

void SourceFileCache::enforceMemoryLimits()
{
  const size_t MAX_HASH_CACHE_ENTRIES = 100;
  const size_t TARGET_CACHE_ENTRIES = 80;

  // Perform LRU eviction if cache is too large
  if (this->hashEntries.size() > MAX_HASH_CACHE_ENTRIES) {
    evictLRUEntries(TARGET_CACHE_ENTRIES);
  }

  // Additional cleanup: remove very old entries regardless of reference count
  std::time_t now = std::time(nullptr);
  const std::time_t MAX_CACHE_AGE = 300; // 5 minutes
  
  auto it = this->hashEntries.begin();
  while (it != this->hashEntries.end()) {
    if (now - it->second.cache_time > MAX_CACHE_AGE && it->second.reference_count == 0) {
      it = this->hashEntries.erase(it);
    } else {
      ++it;
    }
  }
}
