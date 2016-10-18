file(REMOVE_RECURSE
  "fishswarm_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/fishswarm.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
