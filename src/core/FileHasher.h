#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <cstddef>

/*!
  Utility class for calculating file content hashes and dependency hash chains.
  Used by SourceFileCache to implement hash-based caching of parsed files.
*/
class FileHasher
{
public:
  /*!
    Calculate a hash of the file's content combined with its include/use dependencies.
    Returns empty string if file cannot be read.
  */
  static std::string calculateFileHash(const std::string& filename);

  /*!
    Calculate a dependency chain hash that includes the file's own hash
    plus all hashes of files it includes or uses.
  */
  static std::string calculateDependencyChainHash(const std::string& filename, 
                                                  const std::vector<std::string>& dependencies);

  /*!
    Calculate hash of raw content string.
  */
  static std::string calculateContentHash(const std::string& content);

  /*!
    Combine multiple hashes into a single hash string.
  */
  static std::string combineHashes(const std::vector<std::string>& hashes);

private:
  // Convert size_t hash to hex string representation
  static std::string hashToString(std::size_t hash);
  
  // Internal hash function using boost::hash
  static std::size_t hashContent(const std::string& content);
};