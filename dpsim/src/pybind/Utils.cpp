/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/pybind/Utils.h>

CPS::Matrix zeroMatrix(int dim) { return CPS::Matrix::Zero(dim, dim); }

std::string getAttributeList(CPS::IdentifiedObject &obj) {
  std::stringstream output;
  output << std::setiosflags(std::ios::left);
  output << "Attribute list for object with name " << obj.name() << ":"
         << std::endl;
  output << std::setw(30) << "name" << std::setw(20) << "type" << std::setw(15)
         << "size" << std::setw(30) << "value" << std::endl;
  output << std::setw(95) << std::setfill('-') << "" << std::setfill(' ')
         << std::endl;
  for (auto attr : obj.attributes()) {
    std::string name = attr.first;
    std::string type;
    std::string value;
    std::string size;
    if (auto tryReal = std::dynamic_pointer_cast<CPS::Attribute<CPS::Real>>(
            attr.second.getPtr())) {
      type = "Real";
      value = std::to_string(tryReal->get());
      size = "";
    } else if (auto tryComplex =
                   std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(
                       attr.second.getPtr())) {
      type = "Complex";
      value = std::to_string(abs(tryComplex->get())) + "<" +
              std::to_string(arg(tryComplex->get()));
      size = "";
    } else if (auto tryMatrixReal =
                   std::dynamic_pointer_cast<CPS::Attribute<CPS::Matrix>>(
                       attr.second.getPtr())) {
      type = "MatrixReal";
      value = "[...]";
      size = std::to_string(tryMatrixReal->get().rows()) + "x" +
             std::to_string(tryMatrixReal->get().cols());
    } else if (auto tryMatrixComp =
                   std::dynamic_pointer_cast<CPS::Attribute<CPS::MatrixComp>>(
                       attr.second.getPtr())) {
      type = "MatrixComplex";
      value = "[...]";
      size = std::to_string(tryMatrixComp->get().rows()) + "x" +
             std::to_string(tryMatrixComp->get().cols());
    } else if (auto tryString =
                   std::dynamic_pointer_cast<CPS::Attribute<CPS::String>>(
                       attr.second.getPtr())) {
      type = "String";
      value = "\"" + tryString->get() + "\"";
      size = "";
    } else {
      type = "Unknown";
      value = "Unknown";
      size = "Unknown";
    }
    output << std::setw(30) << name << std::setw(20) << type << std::setw(15)
           << size << std::setw(30) << value << std::endl;
  }
  return output.str();
}

void printAttributes(CPS::IdentifiedObject &obj) {
  py::print(getAttributeList(obj));
}

void printAttribute(CPS::IdentifiedObject &obj, std::string attrName) {
  auto attr = obj.attribute(attrName);
  if (auto tryReal =
          std::dynamic_pointer_cast<CPS::Attribute<CPS::Real>>(attr.getPtr())) {
    py::print("Attribute " + attrName + " on object " + obj.name() +
              " has type Real and value " + std::to_string(tryReal->get()));
  } else if (auto tryComplex =
                 std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(
                     attr.getPtr())) {
    std::string value = std::to_string(abs(tryComplex->get())) + "<" +
                        std::to_string(arg(tryComplex->get()));
    py::print("Attribute " + attrName + " on object " + obj.name() +
              " has type Complex and value " + value);
  } else if (auto tryMatrixReal =
                 std::dynamic_pointer_cast<CPS::Attribute<CPS::Matrix>>(
                     attr.getPtr())) {
    std::string size = std::to_string(tryMatrixReal->get().rows()) + "x" +
                       std::to_string(tryMatrixReal->get().cols());
    py::print("Attribute " + attrName + " on object " + obj.name() +
              " has type MatrixReal (size " + size + ") and value:");
    py::print(py::cast(tryMatrixReal->get()));
  } else if (auto tryMatrixComp =
                 std::dynamic_pointer_cast<CPS::Attribute<CPS::MatrixComp>>(
                     attr.getPtr())) {
    std::string size = std::to_string(tryMatrixComp->get().rows()) + "x" +
                       std::to_string(tryMatrixComp->get().cols());
    py::print("Attribute " + attrName + " on object " + obj.name() +
              " has type MatrixComplex (size " + size + ") and value:");
    py::print(py::cast(tryMatrixComp->get()));
  } else if (auto tryString =
                 std::dynamic_pointer_cast<CPS::Attribute<CPS::String>>(
                     attr.getPtr())) {
    py::print("Attribute " + attrName + " on object " + obj.name() +
              " has type String and value \"" + tryString->get() + "\"");
  } else {
    py::print("Could not determine type of attribute " + attrName);
  }
}
