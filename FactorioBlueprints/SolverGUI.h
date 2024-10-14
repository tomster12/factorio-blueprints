#pragma once

#include <SFML/Graphics.hpp>
#include "ListenableEvent.h"
#include "ProblemSolver.h"

using namespace impl;

inline std::string coordToString(int x, int y)
{
	return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

// Most variables are public for ease of development
namespace gui
{
	class SolverGUI;

	class IOElement
	{
	public:
		SolverGUI* gui;
		sf::RectangleShape rect;
		sf::Vector2i gridPos;
		bool isInput = true;
		bool isSelected = false;
		int item = -1;

		IOElement();
		IOElement(SolverGUI* gui, sf::Vector2i gridPos);
		void render();
		void remove();

	private:
		void updateRect();
	};

	class IOInspector
	{
	public:
		SolverGUI* gui;
		IOElement* element = nullptr;
		sf::RectangleShape bgRect;
		sf::Text toggleText;
		sf::RectangleShape toggleRect;
		sf::Text rateText;
		sf::RectangleShape rateUpRect;
		sf::RectangleShape rateDownRect;
		bool isHighlighted = false;
		bool isToggleHighlighted = false;
		bool isRateUpHighlighted = false;
		bool isRateDownHighlighted = false;

		IOInspector();
		IOInspector(SolverGUI* gui);
		void update();
		void render();
		void select(IOElement* element);
		void deselect();
	};

	class SolverGUI
	{
	public:
		sf::RenderWindow* window;
		sf::Font font;
		sf::Vector2i mousePos;
		sf::RectangleShape gridBackground;
		sf::VertexArray gridLines;
		float gridCellSize;
		float gridPosX, gridPosY;
		sf::Vector2i hoveredCell;
		sf::RectangleShape hoveredCellHighlight;
		ListenableEvent<> onGridChangedEvent;
		std::map<std::string, IOElement> blueprintIOElements;
		IOInspector ioInspector;
		ProblemDefinition problemDefinition;
		bool canElementHover = true;
		bool canElementClick = true;

		SolverGUI(sf::RenderWindow* window);
		void handleEvent(sf::Event& event);
		void update();
		void render();

	private:
		void resizeBlueprint(int width, int height);
	};
}
