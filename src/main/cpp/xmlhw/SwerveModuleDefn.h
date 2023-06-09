
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/SwerveModule.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


/// @class SwerveModuleDefn
/// @brief Create a chassis from an XML definition that includes the chassis type, wheel diameter, 
///        wheel based (front to back distance) and track (side to side distance on axle).
class SwerveModuleDefn
{
	public:
		/// @brief construct a SwerveModuleDefn object
		SwerveModuleDefn() = default;

		/// @brief destroy a SwerveModuleDefn object and free the memory
		virtual ~SwerveModuleDefn() = default;

    	/// @brief  Parse the chassie element (and it children).  When this is done a SwerveModule object exists.
		///		   It can be retrieved from the factory.
		/// @param [in]  pugi::xml_node the chassis element in the XML document
    	/// @return std::shared_ptr<SwerveModule> 
		std::shared_ptr<SwerveModule>  ParseXML
		(
			pugi::xml_node      SwerveModuleNode
		);
};
