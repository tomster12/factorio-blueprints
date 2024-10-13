#include <stdlib.h>
#include <map>
#include <memory>
#include "macros.h"
#include "types.h"
#include "ProblemSolver.h"
#include "PlacementPathfinder.h"
#include "PlacementState.h"

using namespace impl;

void checkPathfinding();
void solveExampleProblem1();
void solveExampleProblem2();
void solveExampleProblem3();

int main()
{
	srand(SRAND_SEED);
	solveExampleProblem2();
}

void solveExampleProblem1()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 1.0f, { { 0, 1 } } }; // 1*Item0 -> 1*Item1 @ 1/s

	// 5x5 Blueprint
	// - Input Item0 @ 1/s at (0, 1)
	// - Output Item1 @ at (4, 2)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(5, 5)
		->addInputItem(0, 1.0f, 0, 1)
		->addOutputItem(1, 4, 2)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}

void solveExampleProblem2()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };           // 1*Item0 -> 1*Item1 @ 0.5/s
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } }; // 2*Item0 + 2*Item1 -> 1*Item2 @ 0.5/s

	// 10x10 Blueprint
	// - Input Item0 @ 4/s at (0, 1)
	// - Output Item2 @ at (9, 9)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}

void solveExampleProblem3()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[2] = { 1, 2.0f, { { 0, 1 }, { 1, 2 }} };  // 1*Item0 + 2*Item1 -> 1*Item2 @ 2.0/s
	recipes[3] = { 1, 0.5f, { { 0, 3 }, { 2, 1 } } }; // 3*Item0 + 2*Item1 -> 1*Item3 @ 0.5/s

	// 15x15 Blueprint
	// - Input Item0 @ 2/s at (0, 2)
	// - Input Item1 @ 4/s at (0, 10)
	// - Output Item3 @ at (14, 14)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(15, 15)
		->addInputItem(0, 2.0f, 0, 2)
		->addInputItem(1, 4.0f, 0, 10)
		->addOutputItem(3, 14, 14)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}

void checkPathfinding()
{
	// Initialize pathfinding world, constraints, and goal
	const std::vector<std::vector<bool>> blockedGrid = {
		{ 0, 0, 1, 1, 0, 0, 0 },
		{ 0, 0, 1, 1, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 1, 0 } };
	std::vector<CATEntry> constraints{};
	PFGoal goal{ Coordinate{ 2, 6 }, 0, 0 };

	// Make initial state and perform pathfinding
	auto initialState = new PFState(blockedGrid, constraints, goal, PFData{ Coordinate{ 0, 0 }, BeltType::None, Direction::E });
	auto path = pf::asPathfinding<PFState, PFData>(initialState);

	// Print path
	for (const auto& nodeData : path->nodes) nodeData.print();
}

void checkCBPathfinding()
{
	// Define problem definition from example 1
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };           // 1*Item0 -> 1*Item1 @ 0.5/s
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } }; // 2*Item0 + 2*Item1 -> 1*Item2 @ 0.5/s

	// 10x10 Blueprint
	// - Input Item0 @ 4/s at (0, 1)
	// - Output Item2 @ at (9, 9)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Setup example run configuration with pre-calculated rates
	RunConfig runConfig;
	runConfig.outputAssemblerCount = 1;
	runConfig.itemConfigs[0] = { 0, 0, 0, 0.0f };
	runConfig.itemConfigs[1] = { 1, 2, 1, 0.5f, { { 0, { 1, 0.5f } } } };
	runConfig.itemConfigs[2] = { 2, 1, 1, 0.5f, { { 0, { 1, 1.0f } }, { 1, { 1, 1.0f }}} };

	// Setup local search state with pre-placed assemblers
	PlacementState::InserterInstance none{ -1, false };
	std::vector<PlacementState::AssemblerInstance> assemblers;
	assemblers.push_back({ 1, { 5, 0 }, { none, none, none, none, none, { 1, 0.5f, false }, { 0, 0.5f, true }, none, none, none, none, none } });
	assemblers.push_back({ 1, { 3, 7 }, { none, none, none, { 0, 0.5f, true }, none, none, none, none, none, none, none, { 1, 0.5f, false } } });
	assemblers.push_back({ 2, { 2, 4 }, { none, none, { 0, 1.0f, true }, { 2, 0.5f, false }, none, none, none, none, none, none, { 1, 1.0f, true }, none } });

	// Check that the state is valid
	std::shared_ptr<PlacementState> state = std::make_shared<PlacementState>(problem, runConfig, assemblers);
	state->print();

	// Request cost to trigger pathfinding to check it works
	std::cout << "State fitness: " << state->getFitness() << std::endl;
}
