# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-src")
  file(MAKE_DIRECTORY "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-src")
endif()
file(MAKE_DIRECTORY
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-build"
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix"
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/tmp"
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/src/vorbis-populate-stamp"
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/src"
  "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/src/vorbis-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/src/vorbis-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/vorbis-subbuild/vorbis-populate-prefix/src/vorbis-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
