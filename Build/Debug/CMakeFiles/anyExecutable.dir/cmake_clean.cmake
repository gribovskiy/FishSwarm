file(REMOVE_RECURSE
  "cats_automoc.cpp"
  "anyExecutable_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/anyExecutable.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
