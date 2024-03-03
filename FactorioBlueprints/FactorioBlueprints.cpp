#include <iostream>
#include <tuple>
#include <map>
#include <vector>
#include <set>
#include <stack>
#include <string>

#include "LocalSearch.h"
#include "Pathfinding.h"

#define LOG

struct Coordinate { int x = 0; int y = 0; };

enum class Direction { UP, DOWN, LEFT, RIGHT };

struct RecipeIngredient { int item = -1; int quantity = 0; };

struct Recipe { int quantity = 0; float rate = 0; std::vector<RecipeIngredient> ingredients; };

struct ProblemItemInput { int item = -1; float rate = 0; Coordinate coordinate; };

struct ProblemItemOutput { int item = -1; Coordinate coordinate; };

struct ProblemDefinition
{
	int binWidth = -1;
	int binHeight = -1;
	std::map<int, Recipe> recipes;
	std::map<int, ProblemItemInput> itemInputs;
	ProblemItemOutput itemOutput;
};

class ProblemDefinitionFactory
{
public:
	static std::unique_ptr<ProblemDefinitionFactory> create()
	{
		return std::make_unique<ProblemDefinitionFactory>();
	}

	ProblemDefinitionFactory* setRecipes(std::map<int, Recipe> recipes)
	{
		problemDefinition.recipes = recipes;
		return this;
	}

	ProblemDefinitionFactory* setSize(int binWidth, int binHeight)
	{
		problemDefinition.binWidth = binWidth;
		problemDefinition.binHeight = binHeight;
		return this;
	}

	ProblemDefinitionFactory* addInputItem(int inputItem, float inputRate, int x, int y)
	{
		problemDefinition.itemInputs[inputItem] = { inputItem, inputRate, { x, y } };
		return this;
	}

	ProblemDefinitionFactory* addOutputItem(int itemOutput, int x, int y)
	{
		problemDefinition.itemOutput = { itemOutput, { x, y } };
		return this;
	}

	ProblemDefinition finalise()
	{
		return std::move(problemDefinition);
	}

private:
	ProblemDefinition problemDefinition;
};

// ------------------------------------------------

// Info extracted from recipes about item
// rate relative to output item rate
struct ItemInfo
{
	int item = -1;
	bool isComponent = false;
	float rate = 0.0f;
};

// For a given run how many assemblers are required for an item
// and how many inserters are required for output / each input
struct RunConfigItemInfo
{
	int item = -1;
	int assemblerCount = 0;
	int outputInsertersPerAssembler = 0;
	std::map<int, int> inputInsertersPerAssembler;
};

// A given run of the solver with a set amount of
// output assemblers and the required item info
struct RunConfig
{
	int outputAssemblerCount = 0;
	std::map<int, RunConfigItemInfo> itemInfos;
};

// An inserter in an AsemblerInstance in an LSState
struct InserterInstance
{
	int item = -1;
	bool isInput = false;
	int a = 1;
};

// An assembler placed in an LSState
struct AssemblerInstance
{
	static const std::vector<Direction> inserterDirections;
	static const std::vector<Coordinate> inserterOffsets;

	int item = -1;
	Coordinate coordinate;
	InserterInstance inserters[12];
};

const std::vector<Direction> AssemblerInstance::inserterDirections = {
	Direction::UP, Direction::UP, Direction::UP,
	Direction::RIGHT, Direction::RIGHT, Direction::RIGHT,
	Direction::DOWN, Direction::DOWN, Direction::DOWN,
	Direction::LEFT, Direction::LEFT, Direction::LEFT
};

const std::vector<Coordinate> AssemblerInstance::inserterOffsets = {
	{ 0, -1 }, { 1, -1 }, { 2, -1 },
	{ 3, 0 }, { 3, 1 }, { 3, 2 },
	{ 2, 3 }, { 1, 3 }, { 0, 3 },
	{ -1, 2 }, { -1, 1 }, { -1, 0 }
};

Direction dirOpposite(Direction dir)
{
	switch (dir)
	{
	case Direction::UP: return Direction::DOWN;
	case Direction::DOWN: return Direction::UP;
	case Direction::LEFT: return Direction::RIGHT;
	case Direction::RIGHT: return Direction::LEFT;
	}
	return Direction::UP;
}

Coordinate dirOffset(Direction dir)
{
	switch (dir)
	{
	case Direction::UP: return { 0, -1 };
	case Direction::DOWN: return { 0, 1 };
	case Direction::LEFT: return { -1, 0 };
	case Direction::RIGHT: return { 1, 0 };
	}
	return { 0, 0 };
}

// A state represents a successfully placed set of assemblers / inserters
// A valid state is one which delivers the output item
// The cost of a state is determined by a CB pathfinding algorithm
class LSState : public ls::State<LSState>
{
public:

	static std::shared_ptr<LSState> createRandom(const ProblemDefinition& problem, const RunConfig& runConfig)
	{
		std::vector<AssemblerInstance> assemblers;

		// For each item type
		for (const auto& item : runConfig.itemInfos)
		{
			const RunConfigItemInfo& itemInfo = item.second;
			if (itemInfo.assemblerCount == 0) continue;

			// Randomly place assemblers
			for (int i = 0; i < itemInfo.assemblerCount; i++)
			{
				Coordinate coord;
				coord.x = rand() % (problem.binWidth - 2);
				coord.y = rand() % (problem.binHeight - 2);
				AssemblerInstance assembler{ item.first, coord };

				// Available inserters for this assembler
				std::vector<int> availableInserters(12);
				for (int i = 0; i < 12; i++) availableInserters[i] = i;

				// Randomly place input inserters
				for (const auto& input : itemInfo.inputInsertersPerAssembler)
				{
					for (int i = 0; i < input.second; i++)
					{
						int index = rand() % availableInserters.size();
						int inserterIndex = availableInserters[index];
						availableInserters.erase(availableInserters.begin() + index);
						assembler.inserters[inserterIndex] = InserterInstance{ input.first, true };
					}
				}

				// Randomly place output inserters
				for (int i = 0; i < itemInfo.outputInsertersPerAssembler; i++)
				{
					int index = rand() % availableInserters.size();
					int inserterIndex = availableInserters[index];
					availableInserters.erase(availableInserters.begin() + index);
					assembler.inserters[inserterIndex] = InserterInstance{ item.first, false };
				}

				assemblers.push_back(assembler);
			}
		}

		return std::make_shared<LSState>(problem, runConfig, assemblers);
	}

	LSState(const ProblemDefinition& problem, const RunConfig& runConfig, const std::vector<AssemblerInstance>& assemblers)
		: problem(problem), runConfig(runConfig), assemblers(assemblers)
	{}

	float getCost() override
	{
		if (costCalculated) return cost;

		cost = 0.0f;

		calculateWorld();
		cost += worldCost;

		costCalculated = true;
		return cost;
	}

	std::vector<std::shared_ptr<LSState>> getNeighbors() override
	{
		if (neighborsCalculated) return neighbors;

		neighbors = std::vector<std::shared_ptr<LSState>>();

		for (int i = 0; i < assemblers.size(); i++)
		{
			const AssemblerInstance& assembler = assemblers[i];

			// Neighbour for moving each assembler in each direction
			for (int j = 0; j < 4; j++)
			{
				Direction dir = static_cast<Direction>(j);
				Coordinate offset = dirOffset(dir);
				Coordinate newCoord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };

				if (newCoord.x < 0 || newCoord.x >= problem.binWidth - 3 || newCoord.y < 0 || newCoord.y >= problem.binHeight - 3) continue;

				std::vector<AssemblerInstance> newAssemblers = assemblers;
				newAssemblers[i].coordinate = newCoord;
				neighbors.push_back(getCached(std::make_shared<LSState>(problem, runConfig, newAssemblers)));
			}

			// Neighbour for moving an inserter from 1 slot to another on the same assembler
			for (int j = 0; j < 12; j++)
			{
				const InserterInstance& inserter = assembler.inserters[j];
				if (inserter.item == -1) continue;

				for (int k = 0; k < 12; k++)
				{
					if (j == k) continue;
					const InserterInstance& otherInserter = assembler.inserters[k];
					if (otherInserter.item != -1) continue;

					std::vector<AssemblerInstance> newAssemblers = assemblers;
					newAssemblers[i].inserters[j] = InserterInstance{ -1, false };
					newAssemblers[i].inserters[k] = inserter;
					neighbors.push_back(getCached(std::make_shared<LSState>(problem, runConfig, newAssemblers)));
				}
			}
		}

		neighborsCalculated = true;
		return neighbors;
	}

	bool operator==(LSState& other) const override
	{
		if (assemblers.size() != other.assemblers.size()) return false;

		for (int i = 0; i < assemblers.size(); i++)
		{
			const AssemblerInstance& a = assemblers[i];
			const AssemblerInstance& b = other.assemblers[i];

			if (a.item != b.item || a.coordinate.x != b.coordinate.x || a.coordinate.y != b.coordinate.y) return false;

			for (int j = 0; j < 12; j++)
			{
				const InserterInstance& aInserter = a.inserters[j];
				const InserterInstance& bInserter = b.inserters[j];

				if (aInserter.item != bInserter.item || aInserter.isInput != bInserter.isInput) return false;
			}
		}

		return true;
	}

	bool getValid()
	{
		calculateWorld();
		return isWorldValid;
	}

	void print()
	{
		calculateWorld();

		for (int y = 0; y < problem.binHeight; y++)
		{
			for (int x = 0; x < problem.binWidth; x++)
			{
				if (world[x][y] == -1) std::cout << "\t-";
				else std::cout << "\t" << world[x][y];
			}
			std::cout << std::endl;
		}

		std::cout << std::endl;

		for (const auto& assembler : assemblers)
		{
			std::cout << "Assembler: " << assembler.item << " at (" << assembler.coordinate.x << ", " << assembler.coordinate.y << ")" << std::endl;
			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				if (inserter.item != -1)
				{
					std::cout << "  Inserter: " << i << " item " << inserter.item << " at (" << assembler.coordinate.x + AssemblerInstance::inserterOffsets[i].x << ", " << assembler.coordinate.y + AssemblerInstance::inserterOffsets[i].y << ")" << std::endl;
				}
			}
		}

		std::cout << std::endl;
	}

private:
	const ProblemDefinition& problem;
	const RunConfig& runConfig;
	std::vector<AssemblerInstance> assemblers;

	bool costCalculated = false;
	bool neighborsCalculated = false;
	bool worldCalculated = false;
	bool isWorldValid = false;

	float cost = 0.0f;
	std::vector<std::shared_ptr<LSState>> neighbors;
	std::vector<std::vector<int>> world;
	float worldCost = 0.0f;

	void calculateWorld()
	{
		if (worldCalculated) return;

		world = std::vector<std::vector<int>>(problem.binWidth, std::vector<int>(problem.binHeight, 0));

		for (int x = 0; x < problem.binWidth; x++)
		{
			for (int y = 0; y < problem.binHeight; y++)
			{
				world[x][y] = -1;
			}
		}

		for (const auto& assembler : assemblers)
		{
			for (int x = 0; x < 3; x++)
			{
				for (int y = 0; y < 3; y++)
				{
					int worldX = assembler.coordinate.x + x;
					int worldY = assembler.coordinate.y + y;

					if (world[worldX][worldY] != -1) worldCost += 1.0f;

					world[worldX][worldY] = assembler.item;
				}
			}

			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				if (inserter.item == -1) continue;

				Coordinate offset = AssemblerInstance::inserterOffsets[i];
				Coordinate coord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };

				if (coord.x < 0 || coord.x >= problem.binWidth || coord.y < 0 || coord.y >= problem.binHeight)
				{
					worldCost += 1.0f;
					continue;
				}

				if (world[coord.x][coord.y] != -1)
				{
					worldCost += 1.0f;
				}

				world[coord.x][coord.y] = inserter.item;
			}
		}

		for (const auto& assembler : assemblers)
		{
			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				if (inserter.item == -1) continue;

				Coordinate offset = AssemblerInstance::inserterOffsets[i];
				Coordinate coord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };

				Direction checkDir = AssemblerInstance::inserterDirections[i];
				Coordinate checkOffset = dirOffset(checkDir);
				Coordinate checkCoord = { coord.x + checkOffset.x, coord.y + checkOffset.y };

				if (checkCoord.x < 0 || checkCoord.x >= problem.binWidth || checkCoord.y < 0 || checkCoord.y >= problem.binHeight)
				{
					worldCost += 1.0f;
					continue;
				}

				if (world[checkCoord.x][checkCoord.y] != -1)
				{
					worldCost += 1.0f;
				}
			}
		}

		isWorldValid = false;
		worldCalculated = true;
	}
};

std::vector<std::shared_ptr<LSState>> LSState::cachedStates = std::vector<std::shared_ptr<LSState>>();

class ProblemSolver
{
public:
	static constexpr float MAX_INSERTER_RATE = 4.62f;
	static constexpr float MAX_CONVEYOR_RATE = 45.0f;

	// Produce a solver object with parameters then solve
	static ProblemSolver solve(const ProblemDefinition& problem)
	{
		ProblemSolver solver(problem);
		solver.solve();
		return solver;
	}

private:
	const ProblemDefinition& problem;
	int componentItemCount = -1;
	int bestRunConfig = -1;
	std::map<int, ItemInfo> baseItemInfos;
	std::map<int, RunConfig> possibleRunConfigs;

	ProblemSolver(const ProblemDefinition& problem)
		: problem(problem)
	{}

	// Main logical solve function
	// Run each solve stage sequentially
	void solve()
	{
		#ifdef LOG
		std::cout << "Solving..." << std::endl << std::endl;
		#endif

		unravelRecipes();
		calculateRunConfigs();
		performSearch();
	}

	// Unravel output item recipe to inputs
	// Keep track of total rates relative to a single output rate
	// Fails early if an item cannot be crafted from inputs
	// Populates this->baseItemInfos and componentItemCount
	void unravelRecipes()
	{
		struct SolverRecipeTrace
		{
			int item;
			float rate;
		};

		#ifdef LOG
		std::cout
			<< "Stage: unravelRecipes()" << std::endl
			<< "=========================" << std::endl
			<< std::endl;

		std::cout << "Problem Input Items: " << std::endl;
		for (const auto& entry : problem.itemInputs)
		{
			std::cout << "- (" << entry.first << ") at (" << entry.second.coordinate.x << ", " << entry.second.coordinate.y << ") at " << entry.second.rate << "/s" << std::endl;
		}
		std::cout << std::endl;

		std::cout
			<< "Problem Output Item:" << std::endl
			<< "- (" << problem.itemOutput.item << ") at (" << problem.itemOutput.coordinate.x << ", " << problem.itemOutput.coordinate.y << ")"
			<< std::endl << std::endl;

		std::cout << "Problem Recipes: " << std::endl;
		for (const auto& entry : problem.recipes)
		{
			const int item = entry.first;
			const Recipe& recipe = entry.second;
			std::cout << "- " << "(" << item << ")*" << recipe.quantity << " at " << recipe.rate << "/s = ";
			for (const auto& ingredent : recipe.ingredients)
			{
				std::cout << "(" << ingredent.item << ")*" << ingredent.quantity << " ";
			}
			std::cout << std::endl;
		}

		std::cout << std::endl << "----------" << std::endl << std::endl;
		#endif

		// Keep track of component items and relative rates
		componentItemCount = 0;
		baseItemInfos.clear();
		std::stack<SolverRecipeTrace> recipeTraceStack;

		// Output has a recipe so add to stack with relative rate for 1 assembler
		int outputItem = problem.itemOutput.item;
		if (problem.recipes.find(outputItem) != problem.recipes.end())
		{
			const auto& outputRecipe = problem.recipes.at(outputItem);
			recipeTraceStack.push({ outputItem, outputRecipe.rate * outputRecipe.quantity });
		}

		// Output is an input so add to stack with no info
		else if (problem.itemInputs.find(outputItem) != problem.itemInputs.end())
			recipeTraceStack.push({ outputItem, 0.0f });

		// Output cannot be reached so error
		else throw std::exception(("Output item (" + std::to_string(outputItem) + ") has no recipe nor is an input.").c_str());

		// Process traced items while any left
		while (recipeTraceStack.size() > 0)
		{
			const auto current = recipeTraceStack.top();
			recipeTraceStack.pop();

			// Initialize or update item info for item
			bool isInput = problem.itemInputs.find(current.item) != problem.itemInputs.end();
			if (baseItemInfos.find(current.item) == baseItemInfos.end())
			{
				baseItemInfos[current.item] = { current.item, !isInput, current.rate };
			}
			else baseItemInfos[current.item].rate += current.rate;

			// Skip if is an input
			if (isInput) continue;

			// No recipe therefore impossible problem
			if (problem.recipes.find(current.item) == problem.recipes.end())
				throw std::exception(("- Component item (" + std::to_string(current.item) + ") has no recipe.").c_str());

			// Has a recipe so recurse down
			componentItemCount++;
			const auto& currentRecipe = problem.recipes.at(current.item);
			for (const auto& ingredient : currentRecipe.ingredients)
			{
				recipeTraceStack.push({ ingredient.item, ingredient.quantity * current.rate / currentRecipe.quantity });
			}
		}

		#ifdef LOG
		std::cout << "Solver Items: " << std::endl;
		for (const auto& entry : baseItemInfos)
		{
			std::cout << "- (" << entry.second.item << ") = "
				<< (entry.second.isComponent ? "Component" : "Input") << " "
				<< "( Rate: " << entry.second.rate << " )"
				<< std::endl;
		}
		std::cout << std::endl << std::endl;
		#endif
	}

	// Figure out maximum possible output assembers
	// First check the minimum required assemblers can fit
	// Then calculate max possible, and work backwards
	// Populates this->possibleRunConfigs
	// Return maximum run config
	void calculateRunConfigs()
	{
		#ifdef LOG
		std::cout
			<< "Stage: calculateRunConfigs()" << std::endl
			<< "=============================" << std::endl
			<< std::endl;
		#endif

		// Calculate maximum supported output assemblers
		float maxSupported = 0.0f;
		for (const auto& input : problem.itemInputs)
		{
			const ProblemItemInput& inputItem = input.second;
			const auto& itemInfo = baseItemInfos.at(inputItem.item);
			float itemsSupportedCount = inputItem.rate / itemInfo.rate;
			maxSupported = std::max(maxSupported, itemsSupportedCount);
		}
		int maxSupportedCeil = static_cast<int>(std::ceil(maxSupported));

		// Calculate run configs from max supported to 1
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			RunConfig runConfig;
			runConfig.outputAssemblerCount = i;

			for (const auto& item : baseItemInfos)
			{
				const ItemInfo& itemInfo = item.second;
				RunConfigItemInfo RunConfigItemInfo{ itemInfo.item };

				// If it is a component item calculate assemblers and inserters counts
				if (itemInfo.isComponent)
				{
					const Recipe& itemRecipe = problem.recipes.at(itemInfo.item);
					RunConfigItemInfo.assemblerCount = static_cast<int>(std::ceil(itemInfo.rate * i / (itemRecipe.quantity * itemRecipe.rate)));

					int outputCount = static_cast<int>(std::ceil(itemInfo.rate * i / MAX_INSERTER_RATE));
					RunConfigItemInfo.outputInsertersPerAssembler = outputCount;

					for (const auto& input : itemRecipe.ingredients)
					{
						int inputCount = static_cast<int>(std::ceil(input.quantity * (itemInfo.rate / itemRecipe.quantity) * i / MAX_INSERTER_RATE));
						RunConfigItemInfo.inputInsertersPerAssembler[input.item] = inputCount;
					}
				}

				runConfig.itemInfos[itemInfo.item] = RunConfigItemInfo;
			}

			possibleRunConfigs[i] = runConfig;
		}

		#ifdef LOG
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			const auto& runConfig = possibleRunConfigs[i];
			std::cout << "Run Config with output assemblers = " << runConfig.outputAssemblerCount << std::endl;
			for (const auto& item : runConfig.itemInfos)
			{
				if (item.second.assemblerCount == 0)
				{
					std::cout << "\tInput item: " << item.first << std::endl;
				}
				else
				{
					std::cout << "\tComponent item: " << item.first << std::endl;
					std::cout << "\t  Assemblers: " << item.second.assemblerCount << std::endl;
					std::cout << "\t  Output inserters / assembler: " << item.second.outputInsertersPerAssembler << std::endl;
					for (const auto& itemInput : item.second.inputInsertersPerAssembler)
					{
						std::cout << "\t  Input inserters / assembler for " << itemInput.first << ": " << itemInput.second << std::endl;
					}
				}
			}
			std::cout << std::endl;
		}
		#endif

		// Check space requirements for each
		bestRunConfig = -1;
		size_t availableSpace = problem.binWidth * problem.binHeight - problem.itemInputs.size() - 1;
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			const auto& runConfig = possibleRunConfigs[i];
			size_t requiredSpace = 0;
			for (const auto& item : runConfig.itemInfos)
			{
				requiredSpace += item.second.assemblerCount * 9;
				requiredSpace += item.second.outputInsertersPerAssembler;
				for (const auto& itemInput : item.second.inputInsertersPerAssembler)
				{
					requiredSpace += itemInput.second;
				}
			}

			#ifdef LOG
			std::cout << "Run config " << i << " required space: " << requiredSpace << " / " << availableSpace << std::endl;
			#endif

			// Have found the highest run config so break out
			if (requiredSpace <= availableSpace)
			{
				bestRunConfig = i;
				break;
			}
		}

		#ifdef LOG
		std::cout << "Found best run config: " << bestRunConfig << std::endl << std::endl;
		#endif
	}

	// Perform the main search for a solution
	// Top level local search with placement of assemblers / inserters
	// Lower level conflict based search over orderings of paths
	// Bottom level A* to find paths
	void performSearch()
	{
		#ifdef LOG
		std::cout
			<< "Stage: performSearch()" << std::endl
			<< "=======================" << std::endl
			<< std::endl;
		#endif

		for (int i = bestRunConfig; i > 0; i--)
		{
			const RunConfig& runConfig = possibleRunConfigs.at(i);

			std::shared_ptr<LSState> initialState = LSState::createRandom(problem, runConfig);
			std::shared_ptr<LSState> finalState = ls::hillClimbing(initialState, 100, true);
			//std::shared_ptr<LSState> finalState = ls::simulatedAnnealing(initialState, 2.0f, 0.02f, 100, true);

			#ifdef LOG
			std::cout << "Run config " << i << " final state cost: " << finalState->getCost() << std::endl << std::endl;
			finalState->print();
			#endif
		}
	}
};

// ------------------------------------------------

int main()
{
	srand(0);

	// Define main parameters
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	// Define problem definition
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 6, 6)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}
