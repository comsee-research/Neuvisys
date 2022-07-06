cmake_minimum_required(VERSION 3.0)

project(libserie)

# Crée des variables avec les fichiers à compiler
set(NEUVISYS_LIBSERIE_SOURCES
    comserie.cpp
    Faulhaber.cpp
)
    
set(NEUVISYS_LIBSERIE_HEADERS
    comserie.hpp
    Faulhaber.hpp
)
    
# Notre exécutable
add_library(libserie ${NEUVISYS_LIBSERIE_SOURCES} ${NEUVISYS_LIBSERIE_HEADERS})


set(NEUVISYS_LIBSERIE_SOURCES ${NEUVISYS_LIBSERIE_SOURCES} PARENT_SCOPE)


