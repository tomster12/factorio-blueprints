#include <iostream>
#include "SolverGUI.h"

using namespace impl;

namespace gui
{
	SolverGUI::SolverGUI(sf::RenderWindow* window)
		: window(window), ioInspector(this)
	{
		this->font.loadFromFile("assets/arial.ttf");
		this->gridBackground.setFillColor(sf::Color(199, 166, 125));
		this->gridLines.setPrimitiveType(sf::Lines);
		this->hoveredCellHighlight = sf::RectangleShape(sf::Vector2f(0, 0));
		this->hoveredCellHighlight.setFillColor(sf::Color(255, 255, 255, 100));
		this->hoveredCell = sf::Vector2i(-1, -1);
		this->blueprintIOElements = std::map<std::string, IOElement>();

		// Preset problem definition
		this->resizeBlueprint(10, 10);
	}

	void SolverGUI::update()
	{
		this->canElementHover = true;
		this->canElementClick = true;

		this->mousePos = sf::Mouse::getPosition(*this->window);

		this->ioInspector.update();

		if (this->canElementHover &&
			this->mousePos.x >= this->gridPosX && this->mousePos.x < this->gridPosX + this->problemDefinition.blueprintWidth * this->gridCellSize &&
			this->mousePos.y >= this->gridPosY && this->mousePos.y < this->gridPosY + this->problemDefinition.blueprintHeight * this->gridCellSize)
		{
			this->hoveredCell.x = (int)((this->mousePos.x - this->gridPosX) / this->gridCellSize);
			this->hoveredCell.y = (int)((this->mousePos.y - this->gridPosY) / this->gridCellSize);
			this->hoveredCellHighlight.setPosition(this->gridPosX + this->hoveredCell.x * this->gridCellSize, this->gridPosY + this->hoveredCell.y * this->gridCellSize);
		}
		else this->hoveredCell = sf::Vector2i(-1, -1);
	}

	void SolverGUI::render()
	{
		this->window->clear(sf::Color(42, 43, 46));

		this->window->draw(this->gridBackground);
		this->window->draw(this->gridLines);

		if (this->hoveredCell.x != -1 && this->hoveredCell.y != -1)
		{
			this->window->draw(this->hoveredCellHighlight);
		}

		for (auto& blueprintIOElement : this->blueprintIOElements)
		{
			blueprintIOElement.second.render();
		}

		this->ioInspector.render();
	}

	void SolverGUI::handleEvent(sf::Event& event)
	{
		if (event.type == sf::Event::MouseButtonPressed)
		{
			// Handle IO element events
			if (this->canElementClick && this->hoveredCell.x != -1 && this->hoveredCell.y != -1)
			{
				std::string hoveredCellString = coordToString(this->hoveredCell.x, this->hoveredCell.y);

				if (event.mouseButton.button == sf::Mouse::Left)
				{
					// Left click empty cell so make a new element
					if (this->blueprintIOElements.find(hoveredCellString) == this->blueprintIOElements.end())
					{
						this->blueprintIOElements[hoveredCellString] = IOElement(this, this->hoveredCell);
					}

					// Left clicked cell with element so select with inspector
					this->ioInspector.select(&this->blueprintIOElements[hoveredCellString]);
					this->canElementClick = false;
				}

				// Right click cell with element so remove
				else if (event.mouseButton.button == sf::Mouse::Right)
				{
					if (this->blueprintIOElements.find(hoveredCellString) != this->blueprintIOElements.end())
					{
						this->blueprintIOElements[hoveredCellString].remove();
						this->blueprintIOElements.erase(hoveredCellString);
					}

					this->ioInspector.deselect();
					this->canElementClick = false;
				}
			}

			// Clicking background
			if (this->canElementClick)
			{
				this->ioInspector.deselect();
			}
		}
	}

	void SolverGUI::resizeBlueprint(int width, int height)
	{
		// Update problem definition, grid shapes, and trigger events for other shapes
		this->problemDefinition.blueprintWidth = width;
		this->problemDefinition.blueprintHeight = height;

		const float border = 200;
		const float xBorderPct = 0.5f;
		const float maxSize = this->window->getSize().y - border * 2;
		const float centreX = border * xBorderPct + maxSize * 0.5f;
		const float centreY = this->window->getSize().y * 0.5f;
		const float maxDimension = (float)std::max(width, height);
		this->gridCellSize = maxSize / maxDimension;
		this->gridPosX = centreX - (float)width * this->gridCellSize * 0.5f;
		this->gridPosY = centreY - (float)height * this->gridCellSize * 0.5f;

		this->gridBackground.setSize(sf::Vector2f(width * this->gridCellSize, height * this->gridCellSize));
		this->gridBackground.setPosition(this->gridPosX, this->gridPosY);

		this->gridLines.clear();

		sf::Color col(207, 175, 136);
		for (int i = 1; i < width; i++)
		{
			this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + i * this->gridCellSize, this->gridPosY), col));
			this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + i * this->gridCellSize, this->gridPosY + height * this->gridCellSize), col));
		}
		for (int i = 1; i < height; i++)
		{
			this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX, this->gridPosY + i * this->gridCellSize), col));
			this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + width * this->gridCellSize, this->gridPosY + i * this->gridCellSize), col));
		}

		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX, this->gridPosY), sf::Color::Black));
		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + width * this->gridCellSize, this->gridPosY), sf::Color::Black));

		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + width * this->gridCellSize, this->gridPosY), sf::Color::Black));
		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + width * this->gridCellSize, this->gridPosY + height * this->gridCellSize), sf::Color::Black));

		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX + width * this->gridCellSize, this->gridPosY + height * this->gridCellSize), sf::Color::Black));
		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX, this->gridPosY + height * this->gridCellSize), sf::Color::Black));

		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX, this->gridPosY + height * this->gridCellSize), sf::Color::Black));
		this->gridLines.append(sf::Vertex(sf::Vector2f(this->gridPosX, this->gridPosY), sf::Color::Black));

		this->hoveredCellHighlight.setSize(sf::Vector2f(this->gridCellSize, this->gridCellSize));

		this->onGridChangedEvent.trigger();
	}

	IOElement::IOElement() : gui(nullptr), gridPos(0, 0)
	{}

	IOElement::IOElement(SolverGUI* gui, sf::Vector2i gridPos) : gui(gui), gridPos(gridPos)
	{
		this->rect = sf::RectangleShape(sf::Vector2f(0, 0));
		this->gui->onGridChangedEvent.addListener([&]() { this->updateRect(); });
		this->updateRect();
	}

	void IOElement::render()
	{
		this->gui->window->draw(this->rect);
	}

	void IOElement::remove()
	{}

	void IOElement::updateRect()
	{
		this->rect.setSize(sf::Vector2f(this->gui->gridCellSize, this->gui->gridCellSize));
		this->rect.setPosition(this->gui->gridPosX + this->gridPos.x * this->gui->gridCellSize, this->gui->gridPosY + this->gridPos.y * this->gui->gridCellSize);
		if (isInput) this->rect.setFillColor(sf::Color(252, 217, 88));
		else this->rect.setFillColor(sf::Color(97, 176, 255));
	}

	IOInspector::IOInspector()
		: gui(nullptr)
	{}

	IOInspector::IOInspector(SolverGUI* gui)
		: gui(gui)
	{
		// Setup shapes
		this->bgRect = sf::RectangleShape(sf::Vector2f(300, 600));
		this->bgRect.setFillColor(sf::Color(42, 43, 46));
		this->toggleText = sf::Text("Toggle", gui->font, 20);
	}

	void IOInspector::update()
	{}

	void IOInspector::render()
	{}

	void IOInspector::select(IOElement* element)
	{
		sf::Vector2f elementPos = element->rect.getPosition();
	}

	void IOInspector::deselect()
	{}
}
