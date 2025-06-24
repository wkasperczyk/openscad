#pragma once

#include <string>
#include <ctime>
#include <unordered_map>
#include <vector>

class SourceFile;

/*!
   Caches SourceFiles based on their filenames
 */
class SourceFileCache
{
public:
  static SourceFileCache *instance() { if (!inst) inst = new SourceFileCache; return inst; }

  std::time_t evaluate(const std::string& mainFile, const std::string& filename, SourceFile *& sourceFile);
  SourceFile *lookup(const std::string& filename);
  
  // Hash-based cache methods
  SourceFile *lookupByHash(const std::string& dependencyChainHash);
  bool cacheByHash(const std::string& dependencyChainHash, SourceFile *sourceFile, 
                   const std::vector<std::string>& dependencies);
  std::string calculateDependencyChainHash(const std::string& filename, 
                                          const std::vector<std::string>& dependencies);
  
  // Memory management
  void releaseReference(const std::string& dependencyChainHash);
  void evictLRUEntries(size_t maxEntries = 100);
  void enforceMemoryLimits();
  
  size_t size() const { return this->entries.size(); }
  size_t hashCacheSize() const { return this->hashEntries.size(); }
  void clear();
  void clearHashCache();
  static void clear_markers();

private:
  SourceFileCache() = default;

  static SourceFileCache *inst;

  struct cache_entry {
    SourceFile *file{};
    SourceFile *parsed_file{};                   // the last version parsed for the include list
    std::string cache_id;
    std::time_t mtime{}; // time file last modified
    std::time_t includes_mtime{}; // time the includes last changed
  };
  
  struct hash_cache_entry {
    SourceFile *file{};                          // cached parsed file
    std::vector<std::string> dependencies;      // list of dependency file paths
    std::time_t cache_time{};                    // time this entry was created
    size_t reference_count{};                    // reference counting for memory management
    std::time_t last_accessed{};                 // time this entry was last accessed (for LRU)
  };
  
  std::unordered_map<std::string, cache_entry> entries;
  std::unordered_map<std::string, hash_cache_entry> hashEntries;
};
