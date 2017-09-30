This directory exists and should be the destination for copied header files.
For example, when building uWebSockets, the CMakeLists.txt contains this
directory as include install path, i.e.

  install (FILES <header-files> DESTINATION <this-path>/include/<component>)

Similarly, this path will searched for header files in a CMakeLists.txt of a
SDC-ND project, i.e.

  include_directories(./../resources/include)
