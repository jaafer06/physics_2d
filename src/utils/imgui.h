#pragma once
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "GLFW/glfw3.h"
#include "Eigen/Dense"
#include <string>
#include <iostream>
#include <functional>
#include <string>

namespace utils {

class ImguiCanvas {

friend class ImGuiWrapper;
public:
	ImguiCanvas(unsigned width, unsigned height): width { width }, height { height } {

	}
	
	void setSize(unsigned _width, unsigned _height) {
		width = _width;
		height = _height;
	}

	void drawRectangle(std::array<Eigen::Vector2f, 4> pts, const ImColor& color = { 1.f, 1.f, 1.f }) {
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		toCanvasCoordinate(pts);
		draw_list->AddPolyline((ImVec2*)pts.data(), 4, color, 1, 1);
	}

	void drawFilledRectangle(std::array<Eigen::Vector2f, 4> pts, const ImColor& color = { 1.f, 1.f, 1.f }) {
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		toCanvasCoordinate(pts);
		draw_list->AddConvexPolyFilled((ImVec2*)pts.data(), 4, color);
	}

	void drawLine(std::array<Eigen::Vector2f, 2> pts, const ImColor& color = { 1.f, 1.f, 1.f }, float thickness=1) {
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		toCanvasCoordinate(pts);
		draw_list->AddPolyline((ImVec2*)pts.data(), 2, color, 0, thickness);
	}

	void drawPoint(Eigen::Vector2f p, const ImColor& color = { 1.f, 1.f, 1.f }, float radius = 3) {
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		toCanvasCoordinate(p);
		draw_list->AddCircleFilled((ImVec2&)p, radius, color);
	}

	std::function<void(ImguiCanvas& canvas)> drawCallback;
	Eigen::Vector2f mousePos;

private:
	template<unsigned n>
	void toCanvasCoordinate(std::array<Eigen::Vector2f, n>& pts) {
		std::for_each(pts.begin(), pts.end(), [this](Eigen::Vector2f& p) {toCanvasCoordinate(p); });
	};

	void toCanvasCoordinate(Eigen::Vector2f& p) {
		p.y() = height - p.y();
		p = p + canvasPos;
	};
	
	void render() {
		ImGui::SetNextWindowSize(ImVec2(350, 560), ImGuiCond_FirstUseEver);
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		auto p = ImGui::GetCursorScreenPos();
		canvasPos = { p.x, p.y };
		ImVec2 canvas_size = ImGui::GetContentRegionAvail();        // Resize canvas to what's available 
		ImGui::InvisibleButton("canvas", canvas_size);
		ImVec2 _mousePos = ImVec2(ImGui::GetIO().MousePos.x - canvasPos.x(), ImGui::GetIO().MousePos.y - canvasPos.y());
		mousePos = { _mousePos.x, height - _mousePos.y };
		drawCallback(*this);
	};

	Eigen::Vector2f canvasPos;
	unsigned width{ 0 };
	unsigned height{ 0 };

};

class ImGuiWrapper {
public:
	ImGuiWrapper(GLFWwindow* window, std::string&& glsl_version) {
		// Setup Dear ImGui context
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		// Setup Platform/Renderer bindings
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version.c_str());
		// Setup Dear ImGui style
		ImGui::StyleColorsDark();
	}

	ImguiCanvas& createCanvas(unsigned width, unsigned height) {
		canvas = new ImguiCanvas(width, height);
		return *canvas;
	}

	template<typename Type, int n, int m>
	void display(Eigen::Matrix<Type, n, m>& matrix, const std::string& displayName) {
		const auto callback = [&](std::string& displayName) {
			ImGui::SetNextItemOpen(true, ImGuiTreeNodeFlags_DefaultOpen);
			if (ImGui::TreeNode(displayName.c_str())) {
				if (ImGui::BeginTable("", m)) {
					for (int row = 0; row < n; row++) {
						ImGui::TableNextRow();
						for (int column = 0; column < m; column++) {
							ImGui::TableSetColumnIndex(column);
							ImGui::PushID(column + row * m);
							ImGui::InputFloat("", &matrix(row, column));
							ImGui::PopID();
						}
					}
					ImGui::EndTable();
				}
				ImGui::TreePop();
			}

		};
		named_callbacks.push_back({ callback, displayName });
	}

	template<typename Type, int n, int m>
	void display(const Eigen::Matrix<Type, n, m>& matrix, const std::string&& displayName) {
		const auto callback = [&](std::string& displayName) {
			ImGui::SetNextItemOpen(true, ImGuiTreeNodeFlags_DefaultOpen);
			if (ImGui::TreeNode(displayName.c_str())) {
				if (ImGui::BeginTable("", m)) {
					for (int row = 0; row < n; row++) {
						ImGui::TableNextRow();
						for (int column = 0; column < m; column++) {
							ImGui::TableSetColumnIndex(column);
							ImGui::PushID(column + row * m);
							ImGui::InputFloat("", (float*)&matrix(row, column), NULL, NULL, NULL, ImGuiInputTextFlags_ReadOnly);
							ImGui::PopID();
						}
					}
					ImGui::EndTable();
				}
				ImGui::TreePop();
			}

		};
		named_callbacks.push_back({ callback, displayName });
	}


	template<typename Type, int n>
	void display(Eigen::Matrix<Type, n, 1>& vector, const std::string&& displayName) {
		const auto callback = [&](std::string& displayName) { 
			ImGui::InputScalarN(displayName.c_str(), ImGuiDataType_Float, vector.data(), n);
		};
		named_callbacks.push_back({ callback,  displayName });
	}

	template<typename Type, int n>
	void displaySlider(Eigen::Matrix<Type, n, 1>& vector, const std::string&& displayName) {
		const auto callback = [&](std::string& displayName) {
			ImGui::PushID("slider");
			constexpr float mins[n] = { -10 };
			constexpr float maxs[n] = { 10 };
			ImGui::SliderScalarN(displayName.c_str(), ImGuiDataType_Float, vector.data(), n, mins, maxs);
			ImGui::PopID();

		};
		named_callbacks.push_back({ callback,  displayName });
	}

	template<typename Type, int n>
	void display(const Eigen::Matrix<Type, n, 1>& vector, const std::string&& displayName) {
		const auto callback = [&](std::string& displayName) {
			ImGui::InputScalarN(displayName.c_str(), ImGuiDataType_Float, (float*)vector.data(), n, NULL, NULL, NULL, ImGuiInputTextFlags_ReadOnly);
		};
		named_callbacks.push_back({ callback,  displayName });
	}

	void render() {
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGui::Begin("check da stats man!");
		for (auto& named_callback : named_callbacks) {
			auto& [callback, displayName] = named_callback;
			callback(displayName);
		}
		if (canvas) {
			canvas->render();
		}

		ImGui::End();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

private:
	std::vector<std::tuple<std::function<void(std::string&)>, std::string>> named_callbacks;
	ImguiCanvas* canvas{ nullptr };
};

}
