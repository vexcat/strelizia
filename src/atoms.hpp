#pragma once
#include <string>
void init_atoms();
void install_hawt_atom(const std::string& name, void* what);
void* retrieve_hawt_atom(const std::string& name);