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

// ------------------------

struct ProblemItemInput { int item = -1; float rate = 0; Coordinate coordinate; };

struct ProblemItemOutput { int item = -1; Coordinate coordinate; };

struct ProblemDefinition
{
	int binWidth = -1;
	int binHeight = -1;
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

struct RecipeIngredient { int item = -1; int quantity = 0; };

struct Recipe { int quantity = 0; float rate = 0; std::vector<RecipeIngredient> ingredients; };

// ------------------------

struct ItemInfo
{
	int item = -1;
	bool isComponent = false;
	float rate = 0.0f;
};

struct RunConfigItemInfo
{
	int item = -1;
	int assemblerCount = 0;
	int outputInsertersPerAssembler = 0;
	std::map<int, int> inputInsertersPerAssembler;
};

struct RunConfig
{
	int outputAssemblerCount = 0;
	std::map<int, RunConfigItemInfo> itemInfos;
};

class LSState : public ls::State
{
	// static createRandom() that takes in a RunConfig
	// - Generates assemblers with random inserter placement
	// - Generates bounding boxes for shape outline
	// - Performs bin packing to find oreitnation

	// Get neighbours needs to cache
	// Possible state changes:
	// - Move assembler by 1 square
	// - Move 1 inserter position
	// - Swap assembler recipe

public:
	LSState(int value) : State(value), value(value) {}

	float getCost() override
	{
		return static_cast<float>(value);
	}

	std::vector<ls::StatePtr> getNeighbors() override
	{
		return {
			std::make_shared<LSState>(value - 1),
			std::make_shared<LSState>(value + 1)
		};
	}

private:
	int value;
};

class ProblemSolver
{
public:
	static constexpr float MAX_INSERTER_RATE = 4.62f;
	static constexpr float MAX_CONVEYOR_RATE = 45.0f;

	// Produce a solver object with parameters then solve
	static ProblemSolver solve(const std::map<int, Recipe>& recipes, const ProblemDefinition& problem)
	{
		ProblemSolver solver(recipes, problem);
		solver.solve();
		return solver;
	}

private:
	const std::map<int, Recipe>& recipes;
	const ProblemDefinition& problem;

	int componentItemCount = -1;
	std::map<int, ItemInfo> baseItemInfos;
	std::map<int, RunConfig> possibleRunConfigs;

	ProblemSolver(const std::map<int, Recipe>& recipes, const ProblemDefinition& problem)
		: recipes(recipes), problem(problem)
	{}

	// Main logical solve function
	// Run each solve stage sequentially
	void solve()
	{
		#ifdef LOG
		std::cout << "Solving..." << std::endl << std::endl;
		#endif

		unravelRecipes();
		int currentRunConfig = calculateRunConfigs();
		performSearch(possibleRunConfigs[currentRunConfig]);
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
		for (const auto& entry : recipes)
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
		if (recipes.find(outputItem) != recipes.end())
		{
			const auto& outputRecipe = recipes.at(outputItem);
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
			if (recipes.find(current.item) == recipes.end())
				throw std::exception(("- Component item (" + std::to_string(current.item) + ") has no recipe.").c_str());

			// Has a recipe so recurse down
			componentItemCount++;
			const auto& currentRecipe = recipes.at(current.item);
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
	int calculateRunConfigs()
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
					const Recipe& itemRecipe = recipes.at(itemInfo.item);
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
		int bestRunConfig = -1;
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

		return bestRunConfig;
	}

	// Perform the main search for a solution
	// Top level local search with placement of assemblers / inserters
	// Lower level conflict based search over orderings of paths
	// Bottom level A* to find paths
	void performSearch(const RunConfig& runConfig)
	{
		#ifdef LOG
		std::cout
			<< "Stage: performSearch()" << std::endl
			<< "=======================" << std::endl
			<< std::endl;
		#endif

		ls::StatePtr _result = ls::hillClimbing(std::make_shared<LSState>(0), 50);
		std::shared_ptr<LSState> result = std::static_pointer_cast<LSState>(_result);
		std::cout << "Result: " << result->getCost() << std::endl;
	}
};

// ------------------------

int main()
{
	// Define main parameters
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	// Define problem definition
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setSize(7, 7)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 6, 6)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(recipes, problem);
}
