#pragma once
#include <glad/glad.h>
#include <string>
#include <fstream>
#include <iostream>

namespace utils {

	GLuint loadShader(std::string&& filePath, GLenum shaderType) {
		GLuint shader = glCreateShader(shaderType);
		std::string shaderCode;
		std::ifstream fileStream(filePath, std::ios::in);
		if (fileStream.is_open()) {
			std::stringstream sstr;
			sstr << fileStream.rdbuf();
			shaderCode = sstr.str();
			fileStream.close();
		} else {
			throw ("could not open file " + filePath);
		}

		std::cout << "Compiling shader :" << filePath << std::endl;
		char const* shaderCodePointer = shaderCode.c_str();
		glShaderSource(shader, 1, &shaderCodePointer, NULL);
		glCompileShader(shader);

		GLint Result;
		int InfoLogLength;

		glGetShaderiv(shader, GL_COMPILE_STATUS, &Result);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength > 0) {
			std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
			glGetShaderInfoLog(shader, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
			printf("%s\n", &VertexShaderErrorMessage[0]);
			throw "error while compiling shader " + filePath;
		}
		return shader;
	}

	template<typename ...Args>
	void linkShadersHelper(GLuint ProgramID, GLuint shader, Args... args) {
		glAttachShader(ProgramID, shader);
		linkShadersHelper(ProgramID, args...);
	}
	
	template<>
	void linkShadersHelper(GLuint ProgramID, GLuint shader) {
		glAttachShader(ProgramID, shader);
	}

	template<typename ...Args>
	GLuint linkShaders(Args... args) {
		GLuint ProgramID = glCreateProgram();

		linkShadersHelper(ProgramID, args...);
		glLinkProgram(ProgramID);

		GLint Result;
		int InfoLogLength;
		glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
		glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength > 0) {
			std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
			glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
			printf("%s\n", &ProgramErrorMessage[0]);
			throw "error while linking shaders";
		}

		return ProgramID;
	}



}
