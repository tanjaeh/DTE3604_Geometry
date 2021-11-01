/**********************************************************************************
**
** Copyright (C) 1994 Narvik University College
** Contact: GMlib Online Portal at http://episteme.hin.no
**
** This file is part of the Geometric Modeling Library, GMlib.
**
** GMlib is free software: you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** GMlib is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with GMlib.  If not, see <http://www.gnu.org/licenses/>.
**
**********************************************************************************/



#include "gmcomputeshader.h"


namespace GMlib {

namespace GL {

  ComputeShader::ComputeShader() {}

  void ComputeShader::create() {
    Shader::create(GL_COMPUTE_SHADER);
  }

  void ComputeShader::create(const std::string &name) {
    Shader::create( name, GL_COMPUTE_SHADER );
  }

} // END namespace GL

} // END namespace GMlib