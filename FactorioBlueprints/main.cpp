#include "SolverGUI.h"
#include "macros.h"
#include "ProblemSolver.h"

void solveExampleProblem();
void runSolverGUI();

int main()
{
	runSolverGUI();
}

void runSolverGUI()
{
	// Initialize window
	sf::RenderWindow* window;
	sf::VideoMode windowMode = sf::VideoMode::getDesktopMode();
	windowMode.width = 1600;
	windowMode.height = 1000;
	std::string title = "Factorio Blueprints";
	bool fullscreen = false;
	unsigned framerateLimit = 60;
	bool verticalSyncEnabled = false;
	if (fullscreen) window = new sf::RenderWindow(windowMode, title, sf::Style::Fullscreen);
	else window = new sf::RenderWindow(windowMode, title, sf::Style::Titlebar | sf::Style::Close);
	window->setFramerateLimit(framerateLimit);
	window->setVerticalSyncEnabled(verticalSyncEnabled);

	// Setup SolverGUI
	gui::SolverGUI solverGUI(window);

	// Main event loop
	sf::Event sfEvent;
	while (window->isOpen())
	{
		while (window->pollEvent(sfEvent))
		{
			if (sfEvent.type == sf::Event::Closed)
			{
				window->close();
			}
			else solverGUI.handleEvent(sfEvent);
		}

		solverGUI.update();
		solverGUI.render();
		window->display();
	}

	// Cleanup
	delete window;
}

void solveExampleProblem()
{
	srand(SRAND_SEED);

	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };           // 1*Item0 -> 1*Item1 @ 0.5/s
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } }; // 2*Item0 + 2*Item1 -> 1*Item2 @ 0.5/s

	// 8x8 Blueprint
	// - Input Item0 @ 4/s at (0, 1)
	// - Output Item2 @ at (7,7)
	ProblemDefinition problem;
	problem.recipes = recipes;
	problem.blueprintWidth = 8;
	problem.blueprintHeight = 8;
	problem.itemInputs[0] = { 0, 4.0f, { 0, 1 } };
	problem.itemOutput = { 2, { 7, 7 } };

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}
