# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/arthu/.espressif/frameworks/esp-idf-v4.4.6/components/bootloader/subproject"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix/tmp"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix/src"
  "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/arthu/Downloads/EC444/Hua-Arthur/skills/cluster-0/06/code/echo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
