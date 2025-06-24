#include "core/FileHasher.h"
#include "core/StatCache.h"
#include "utils/printutils.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <boost/functional/hash.hpp>

std::string FileHasher::calculateFileHash(const std::string& filename)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    return "";
  }

  // Read entire file content
  std::string content((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  file.close();

  // Get file stats for additional entropy
  struct stat st;
  if (StatCache::stat(filename, st) != 0) {
    return "";
  }

  // Combine content hash with file size for robustness
  std::size_t contentHash = hashContent(content);
  std::size_t combinedHash = contentHash;
  boost::hash_combine(combinedHash, static_cast<std::size_t>(st.st_size));

  return hashToString(combinedHash);
}

std::string FileHasher::calculateDependencyChainHash(const std::string& filename,
                                                     const std::vector<std::string>& dependencies)
{
  std::vector<std::string> allHashes;
  
  // Add the main file's hash
  std::string mainFileHash = calculateFileHash(filename);
  if (mainFileHash.empty()) {
    return "";
  }
  allHashes.push_back(mainFileHash);

  // Add hashes of all dependencies
  for (const auto& dep : dependencies) {
    std::string depHash = calculateFileHash(dep);
    if (depHash.empty()) {
      // If any dependency can't be hashed, the chain is invalid
      return "";
    }
    allHashes.push_back(depHash);
  }

  return combineHashes(allHashes);
}

std::string FileHasher::calculateContentHash(const std::string& content)
{
  return hashToString(hashContent(content));
}

std::string FileHasher::combineHashes(const std::vector<std::string>& hashes)
{
  // Sort hashes for consistent ordering regardless of dependency order
  std::vector<std::string> sortedHashes = hashes;
  std::sort(sortedHashes.begin(), sortedHashes.end());

  std::size_t combinedHash = 0;
  for (const auto& hash : sortedHashes) {
    // Convert hex string back to size_t for combining
    std::size_t hashValue = std::stoull(hash, nullptr, 16);
    boost::hash_combine(combinedHash, hashValue);
  }

  return hashToString(combinedHash);
}

std::string FileHasher::hashToString(std::size_t hash)
{
  std::ostringstream ss;
  ss << std::hex << hash;
  return ss.str();
}

std::size_t FileHasher::hashContent(const std::string& content)
{
  // Use boost::hash which is already available in the codebase
  boost::hash<std::string> hasher;
  return hasher(content);
}