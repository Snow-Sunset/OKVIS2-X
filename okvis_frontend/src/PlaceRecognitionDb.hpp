// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <okvis/Frontend.hpp>
#include <DBoW2/TemplatedDatabase.h>
#include <DBoW2/FBrisk.hpp>
namespace okvis {
/// \brief Opaque structure to use DBoW for loop closure.
class Frontend::DBoW {
 public:
  /// \brief Constructor with Vocabulary directory.
  /// \param dBowVocDir Vocabulary directory.
  DBoW(const std::string& dBowVocDir)
      : vocabulary(dBowVocDir+"/small_voc.yml.gz"),
        // false = do not use direct index.
        database(vocabulary, false, 0)
  {
  }
  /// \brief Constructor with Vocabulary.
  /// \param dBowVoc Vocabulary.
  DBoW(const DBoW2::TemplatedVocabulary<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk>& dBowVoc)
      : vocabulary(dBowVoc),
      // false = do not use direct index.
      database(vocabulary, false, 0)
  {
  }

  DBoW2::TemplatedVocabulary<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> vocabulary; ///< BRISK Voc.
  DBoW2::TemplatedDatabase<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> database; ///< BRISK Database.
  std::vector<uint64> poseIds; ///< The multiframe IDs corresponding to the dBow ones.

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
