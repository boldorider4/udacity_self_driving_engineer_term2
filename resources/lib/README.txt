This directory exists and should be the destination for built libraries.
For example, when building uWebSockets, the CMakeLists.txt contains this
directory as library install path, i.e.

  install (TARGETS <target-name> DESTINATION <this-path>/lib)

Similarly, this path will be linked against in a CMakeLists.txt of a SDC-ND
project, i.e.

  link_directories(./../resources/lib)
