#include <iostream>
#include <map>
#include <stack>
#include <string>
#include "ProblemSolver.h"
#include "types.h"
#include "log.h"
#include "global.h"
#include "macros.h"
#include "LSState.h"

std::unique_ptr<ProblemDefinitionFactory> ProblemDefinitionFactory::create()
{
	return std::make_unique<ProblemDefinitionFactory>();
}

ProblemDefinitionFactory* ProblemDefinitionFactory::setRecipes(std::map<int, Recipe> recipes)
{
	problemDefinition.recipes = recipes;
	return this;
}

ProblemDefinitionFactory* ProblemDefinitionFactory::setSize(int blueprintWidth, int blueprintHeight)
{
	problemDefinition.blueprintWidth = blueprintWidth;
	problemDefinition.blueprintHeight = blueprintHeight;
	return this;
}

ProblemDefinitionFactory* ProblemDefinitionFactory::addInputItem(int inputItem, float inputRate, int x, int y)
{
	problemDefinition.itemInputs[inputItem] = { inputItem, inputRate, { x, y } };
	return this;
}

ProblemDefinitionFactory* ProblemDefinitionFactory::addOutputItem(int itemOutput, int x, int y)
{
	problemDefinition.itemOutput = { itemOutput, { x, y } };
	return this;
}

ProblemDefinition ProblemDefinitionFactory::finalise()
{
	return std::move(problemDefinition);
}

ProblemSolver ProblemSolver::solve(const ProblemDefinition& problem)
{
	// Produce a solver object with parameters then solve
	ProblemSolver solver(problem);
	solver.solve();
	return solver;
}

ProblemSolver::ProblemSolver(const ProblemDefinition& problem)
	: problem(problem)
{}

void ProblemSolver::solve()
{
	// Main logical solve function
	// Run each solve stage sequentially

	LOG(SOLVER, "Solving...\n\n");

	unravelRecipes();
	calculateRunConfigs();
	performSearch();
}

void ProblemSolver::unravelRecipes()
{
	// Unravel output item recipe to inputs
	// Keep track of total rates relative to a single output rate
	// Fails early if an item cannot be crafted from inputs
	// Populates this->baseItemInfos and componentItemCount

	struct RecipeTrace
	{
		int item;
		float rate;
	};

	// LOGGING: Print out problem input items, output item, and recipes
	LOG(SOLVER, "Stage: unravelRecipes()\n"
		<< "=========================\n\n");
	LOG(SOLVER, "Problem Input Items: \n");
	for (const auto& entry : problem.itemInputs)
	{
		LOG(SOLVER, "- (" << entry.first << ") at (" << entry.second.coordinate.x << ", " << entry.second.coordinate.y << ") at " << entry.second.rate << "/s\n");
	}
	LOG(SOLVER, "\n");
	LOG(SOLVER,
		"Problem Output Item:\n"
		<< "- (" << problem.itemOutput.item << ") at "
		<< "(" << problem.itemOutput.coordinate.x << ", " << problem.itemOutput.coordinate.y << ")\n\n");

	LOG(SOLVER, "Problem Recipes: \n");
	for (const auto& entry : problem.recipes)
	{
		const int item = entry.first;
		const Recipe& recipe = entry.second;
		LOG(SOLVER, "- " << "(" << item << ")*" << recipe.quantity << " at " << recipe.rate << "/s = ");
		for (const auto& ingredent : recipe.ingredients)
		{
			LOG(SOLVER, "(" << ingredent.item << ")*" << ingredent.quantity << " ");
		}
		LOG(SOLVER, "\n");
	}
	LOG(SOLVER, "\n----------\n\n");

	// Keep track of component items and relative rates
	componentItemCount = 0;
	baseItemInfos.clear();
	std::stack<RecipeTrace> traceStack;

	// Output has a recipe so add to stack with relative rate for 1 assembler
	int outputItem = problem.itemOutput.item;
	if (problem.recipes.find(outputItem) != problem.recipes.end())
	{
		const auto& outputRecipe = problem.recipes.at(outputItem);
		traceStack.push({ outputItem, outputRecipe.rate * outputRecipe.quantity });
	}

	// Output is an input so add to stack with no info
	else if (problem.itemInputs.find(outputItem) != problem.itemInputs.end())
		traceStack.push({ outputItem, 0.0f });

	// Output cannot be reached so error
	else throw std::exception(("Output item (" + std::to_string(outputItem) + ") has no recipe nor is an input.").c_str());

	// Process traced items while any left
	while (traceStack.size() > 0)
	{
		const auto current = traceStack.top();
		traceStack.pop();

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
			traceStack.push({ ingredient.item, ingredient.quantity * current.rate / currentRecipe.quantity });
		}
	}

	LOG(SOLVER, "Solver Items: \n");
	for (const auto& entry : baseItemInfos)
	{
		LOG(SOLVER, "- (" << entry.second.item << ") = "
			<< (entry.second.isComponent ? "Component" : "Input") << " "
			<< "( Rate: " << entry.second.rate << " )\n");
	}
	LOG(SOLVER, "\n\n");
}

void ProblemSolver::calculateRunConfigs()
{
	// Figure out maximum possible output assembers
	// First check the minimum required assemblers can fit
	// Then calculate max possible, and work backwards
	// Populates this->possibleRunConfigs
	// Return maximum run config

	LOG(SOLVER,
		"Stage: calculateRunConfigs()\n"
		<< "=============================\n\n");

	// Calculate maximum supported output assemblers
	float maxSupported = 0.0f;
	for (const auto& input : problem.itemInputs)
	{
		const auto& inputItem = input.second;
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

		// For each item produce an item config
		for (const auto& itemPair : baseItemInfos)
		{
			const ItemInfo& itemInfo = itemPair.second;
			RunConfig::ItemConfig itemConfig{ itemInfo.item };

			// If it is a component item calculate assemblers and inserters counts
			if (itemInfo.isComponent)
			{
				const Recipe& itemRecipe = problem.recipes.at(itemInfo.item);

				float totalRate = itemInfo.rate * i;
				float singleRate = itemRecipe.rate * itemRecipe.quantity;
				int assemblerCount = static_cast<int>(std::ceil(totalRate / singleRate));
				float realSingleRate = totalRate / assemblerCount;
				int outputCount = static_cast<int>(std::ceil(realSingleRate / MAX_INSERTER_RATE));
				float singleOutputRate = realSingleRate / outputCount;

				itemConfig.assemblerCount = assemblerCount;
				itemConfig.outputInserterRequirement = { outputCount, singleOutputRate };

				for (const auto& input : itemRecipe.ingredients)
				{
					float totalInputRate = realSingleRate * (input.quantity / itemRecipe.quantity);
					int inputCount = static_cast<int>(std::ceil(totalInputRate / MAX_INSERTER_RATE));
					float singleInputRate = totalInputRate / inputCount;

					itemConfig.inputInserterRequirements[input.item] = { inputCount, singleInputRate };
				}
			}

			runConfig.itemConfigs[itemInfo.item] = itemConfig;
		}

		possibleRunConfigs[i] = runConfig;
	}

	// LOGGING: Print out run configs
	for (int i = maxSupportedCeil; i > 0; i--)
	{
		const auto& runConfig = possibleRunConfigs[i];
		LOG(SOLVER, "Run Config with output assemblers = " << runConfig.outputAssemblerCount << "\n");
		for (const auto& item : runConfig.itemConfigs)
		{
			if (item.second.assemblerCount == 0)
			{
				LOG(SOLVER, "\tInput item: " << item.first << "\n");
			}
			else
			{
				LOG(SOLVER, "\tComponent item: " << item.first << "\n");
				LOG(SOLVER, "\t  Assemblers: " << item.second.assemblerCount << "\n");
				LOG(SOLVER, "\t  Output inserters / assembler: " << item.second.outputInserterRequirement.count << " @ " << item.second.outputInserterRequirement.rate << "/s\n");
				for (const auto& inputRequirement : item.second.inputInserterRequirements)
				{
					LOG(SOLVER, "\t  Input inserters / assembler for " << inputRequirement.first << ": " << inputRequirement.second.count << " @ " << inputRequirement.second.rate << "/s\n");
				}
			}
		}
		LOG(SOLVER, "\n");
	}

	// Check space requirements for each
	bestRunConfig = -1;
	size_t availableSpace = problem.blueprintWidth * problem.blueprintHeight - problem.itemInputs.size() - 1;
	for (int i = maxSupportedCeil; i > 0; i--)
	{
		const auto& runConfig = possibleRunConfigs[i];
		size_t requiredSpace = 0;
		for (const auto& itemConfig : runConfig.itemConfigs)
		{
			requiredSpace += itemConfig.second.assemblerCount * 9;
			requiredSpace += itemConfig.second.assemblerCount * itemConfig.second.outputInserterRequirement.count * 2;
			for (const auto& inputRequirement : itemConfig.second.inputInserterRequirements)
			{
				requiredSpace += itemConfig.second.assemblerCount * inputRequirement.second.count * 2;
			}
		}

		LOG(SOLVER, "Run config " << i << " required space: " << requiredSpace << " / " << availableSpace << "\n");

		// Have found the highest run config so break out
		if (requiredSpace <= availableSpace)
		{
			bestRunConfig = i;
			break;
		}
	}

	LOG(SOLVER, "Found best run config: " << bestRunConfig << "\n\n");
}

void ProblemSolver::performSearch()
{
	// Perform the main search for a solution
	// Top level local search with placement of assemblers / inserters
	// Lower level conflict based search over orderings of paths
	// Bottom level A* to find paths

	LOG(SOLVER, "Stage: performSearch()\n"
		<< "=======================\n\n");

	// for (int i = bestRunConfig; i > 0; i--)
	for (int i = 1; i <= bestRunConfig; i++)
	{
		LOG(SOLVER, "--- Evaluating run config " << i << " ---\n\n");

		const RunConfig& runConfig = possibleRunConfigs.at(i);

		Global::evalCountCBP = 0;
		Global::evalCountPF = 0;
		Global::evalCountLS = 0;

		std::shared_ptr<LSState> initialState = LSState::createRandom(problem, runConfig);
		#if USE_ANNEALING
		std::shared_ptr<LSState> finalState = ls::simulatedAnnealing(initialState, ANNEALING_TEMP, ANNEALING_COOLING, ANNEALING_ITERATIONS);
		#else
		std::shared_ptr<LSState> finalState = ls::hillClimbing(initialState, HILLCLIMBING_ITERATIONS);
		#endif

		LOG(SOLVER, "Finished evaluation, summary:\n\n");
		LOG(SOLVER, "- LSState Evaluation count: " << Global::evalCountLS << "\n");
		LOG(SOLVER, "- CBPathfinder Evaluation count: " << Global::evalCountCBP << "\n");
		LOG(SOLVER, "- PFState Evaluation count: " << Global::evalCountPF << "\n");
		LOG(SOLVER, "- Final state fitness: " << finalState->getFitness() << "\n\n");
		if (LOG_SOLVER_ENABLED) finalState->print();
	}
}
