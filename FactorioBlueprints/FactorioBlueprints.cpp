
#include <iostream>
#include <tuple>
#include <map>
#include <vector>
#include <set>
#include <stack>
#include <string>

#define LOG

// ---------------------------------------------------------------------------------

struct Coordinate { int x = 0; int y = 0; };

struct ProblemItemInput { int item = -1; float rate = 0; Coordinate coordinate; };

struct ProblemItemOutput { int item = -1; Coordinate coordinate; };

struct ProblemDefinition
{
	int binWidth = -1;
	int binHeight = -1;
	std::map<int, ProblemItemInput> itemInputs;
	ProblemItemOutput itemOutput;
};

struct RecipeIngredient { int item = -1; int quantity = 0; };

struct Recipe { int quantity = 0; float rate = 0; std::vector<RecipeIngredient> ingredients; };

struct SolverItemInfo
{
	int item;
	bool isComponent;
	float relativeRate;
	float relativeAssemblerCountRaw;
	int relativeAssemblerCount;
};

// ---------------------------------------------------------------------------------

class ProblemDefinitionFactory
{
public:
	static std::unique_ptr<ProblemDefinitionFactory> newProblemDefinition()
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

class ProblemSolver
{
public:
	// Produce a solver object with parameters then solve
	static ProblemSolver solve(const std::map<int, Recipe>& recipes, const ProblemDefinition& problem)
	{
		// Initialize solver, run, return
		ProblemSolver solver(recipes, problem);
		solver.solve();
		return solver;
	}

private:
	// Main parameters
	const std::map<int, Recipe>& recipes;
	const ProblemDefinition& problem;

	// Internal state
	int componentItemCount = 0;
	std::map<int, SolverItemInfo> itemInfos;

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

		// Run solve steps
		unravelRecipes();
	}

	// Unravel output item recipe to inputs
	// Keep track of total rates relative to output rate
	// Fails early if an item cannot be crafted from inputs
	void unravelRecipes()
	{
		#ifdef LOG
		std::cout << "Stage: unravelRecipes()" << std::endl
			<< "------------------------" << std::endl
			<< std::endl;
		
		std::cout << "Problem Input Items: " << std::endl;
		for (const auto& entry : problem.itemInputs)
		{
			std::cout << "- (" << entry.first << ") at (" << entry.second.coordinate.x << ", " << entry.second.coordinate.y << ") at " << entry.second.rate << "/s" << std::endl;
		}
		std::cout << std::endl;

		std::cout << "Problem Output Item:" << std::endl
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
		std::cout << std::endl;
		#endif

		// Keep track of component items and relative rates
		componentItemCount = 0;
		itemInfos.clear();
		std::stack<std::tuple<int, float>> recipeTraceStack;

		// Add output item with base rate equal to 1 assembler
		int outputItem = problem.itemOutput.item;
		if (recipes.find(outputItem) == recipes.end() && problem.itemInputs.find(outputItem) == problem.itemInputs.end())
			throw std::exception(("Output item (" + std::to_string(outputItem) + ") has no recipe.").c_str());
		auto outputRecipe = recipes.at(outputItem);
		float outputBaseRate = outputRecipe.rate * outputRecipe.quantity;
		recipeTraceStack.push({ outputItem, outputBaseRate });

		// Process traced items while any left
		while (recipeTraceStack.size() > 0)
		{
			const auto recipeTrace = recipeTraceStack.top();
			recipeTraceStack.pop();
			int currentItem = std::get<0>(recipeTrace);
			float currentRelativeRate = std::get<1>(recipeTrace);

			// Update item info for item
			bool isInput = problem.itemInputs.find(currentItem) != problem.itemInputs.end();
			if (itemInfos.find(currentItem) == itemInfos.end()) itemInfos[currentItem] = { currentItem, !isInput, 0.0f, 0.0f };
			itemInfos.at(currentItem).relativeRate += currentRelativeRate;

			// Skip if is an input item
			if (isInput) continue;

			// Has no recipe therefore impossible problem
			if (recipes.find(currentItem) == recipes.end())
				throw std::exception(("- Component item (" + std::to_string(currentItem) + ") has no recipe.").c_str());

			// Has a recipe so recurse down
			componentItemCount++;
			auto currentRecipe = recipes.at(currentItem);
			for (const auto& ingredient : currentRecipe.ingredients)
			{
				recipeTraceStack.push({ ingredient.item, ingredient.quantity * currentRelativeRate / currentRecipe.quantity });
			}
		}

		// Fill out item info
		for (auto& entry : itemInfos)
		{
			SolverItemInfo& info = entry.second;
			if (info.isComponent)
			{
				const Recipe& recipe = recipes.at(info.item);
				info.relativeAssemblerCountRaw = info.relativeRate / (recipe.rate * recipe.quantity);;
				info.relativeAssemblerCount = (int)ceil(info.relativeAssemblerCountRaw);
			}
		}

		#ifdef LOG
		std::cout << "Solver Items: " << std::endl;
		for (const auto& entry : itemInfos)
		{
			std::cout << "- (" << entry.second.item << ") = "
				<< (entry.second.isComponent ? "Component" : "Input") << " "
				<< "(Rate: " << entry.second.relativeRate << ", "
				<< "Assemblers: " << entry.second.relativeAssemblerCount << ")"
				<< std::endl;
		}
		std::cout << std::endl;
		#endif
	}

	// Check the minimum required assemblers can fit
	// Sanity check then bin packing check
	void checkMinimumSpace()
	{
		#ifdef LOG
		std::cout << "Stage: checkMinimumSpace()" << std::endl << std::endl;
		#endif

		// Calculate space required
		int blueprintSpace = problem.binWidth * problem.binHeight;
		int minimumSpace = componentItemCount * 10;

		// Cannot fit minimum grid space
		if (blueprintSpace < minimumSpace)
		{
			throw std::exception(("Cannot fit minimumSpace " + std::to_string(minimumSpace) + " into blueprintSpace " + std::to_string(blueprintSpace)).c_str());
		}

		// TODO: Perform bin packing to check if minimum can fit
	}
};

// ---------------------------------------------------------------------------------

int main()
{
	// Define main parameters
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	// Define problem definition
	ProblemDefinition problem = ProblemDefinitionFactory::newProblemDefinition()
		->setSize(4, 4)
		->addInputItem(0, 2.0f, 0, 0)
		->addOutputItem(2, 3, 3)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(recipes, problem);
}
