cmake_minimum_required(VERSION 3.0)

project(libserie)

# Crée des variables avec les fichiers à compiler
set(SRCS
    comserie.cpp
    Faulhaber.cpp
    )
    
set(HEADERS
    comserie.hpp
    Faulhaber.hpp
    )
    
# Notre exécutable
add_library(libserie ${SRCS} ${HEADERS})



