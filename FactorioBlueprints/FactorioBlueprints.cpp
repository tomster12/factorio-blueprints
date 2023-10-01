
#include <iostream>
#include <tuple>
#include <map>
#include <vector>
#include <set>
#include <stack>
#include <string>

struct Coordinate { int x = 0; int y = 0; };
struct ProblemItemInput { int item = -1; float rate = 0; Coordinate coordinate; };
struct ProblemItemOutput { int item = -1; Coordinate coordinate; };
struct ProblemDefinition
{
	int binWidth = -1;
	int binHeight = -1;
	std::vector<ProblemItemInput> itemInputs;
	ProblemItemOutput itemOutput;
};
struct RecipeIngredient { int item = -1; int quantity = 0; };
struct Recipe { int quantity = 0; float rate = 0; std::vector<RecipeIngredient> ingredients; };
struct ItemRateInfo { float relativeRate; float relativeAssemblerCount; };

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
		problemDefinition.itemInputs.push_back({ inputItem, inputRate, { x, y } });
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
	ProblemSolver(const std::map<int, Recipe>& recipes)
		: recipes(recipes)
	{}

	void solve(const ProblemDefinition& problem)
	{
		// Run solve steps
		this->problem = problem;
		std::cout << "Solving..." << std::endl << std::endl;
		unravelRecipes();

		// Log unique items
		std::cout << "Component Items: ";
		for (int componentItem : componentItems) std::cout << componentItem << " ";
		std::cout << std::endl;
	}

private:
	const std::map<int, Recipe>& recipes;
	ProblemDefinition problem;
	std::set<int> componentItems;
	std::map<int, ItemRateInfo> itemRelativeRates;

	// Unravel output item recipe to inputs
	// Fails early if an item cannot be crafted from inputs
	void unravelRecipes()
	{
		// Produce a set of input items
		std::set<int> itemInputs;
		for (int i = 0; i < problem.itemInputs.size(); i++) itemInputs.insert(problem.itemInputs[i].item);

		// Keep track of component items and relative rates
		componentItems.clear();
		itemRelativeRates.clear();
		std::stack<std::tuple<int, float>> recipeTraceStack;

		// Add output item with base rate equal to 1 assembler
		int outputItem = problem.itemOutput.item;
		auto outputRecipe = recipes.at(outputItem);
		float outputBaseRate = outputRecipe.rate * outputRecipe.quantity;
		recipeTraceStack.push({ outputItem, outputBaseRate });

		// Process traced items while any left
		while (recipeTraceStack.size() > 0)
		{
			auto recipeTrace = recipeTraceStack.top();
			recipeTraceStack.pop();
			int currentItem = std::get<0>(recipeTrace);
			float currentRelativeRate = std::get<1>(recipeTrace);

			// Update relative rate for items
			if (itemRelativeRates.find(currentItem) == itemRelativeRates.end()) itemRelativeRates[currentItem] = {};
			itemRelativeRates.at(currentItem).relativeRate += currentRelativeRate;

			// Skip if is an input item
			if (itemInputs.find(currentItem) != itemInputs.end()) continue;

			// Has no recipe therefore impossible problem
			if (recipes.find(currentItem) == recipes.end()) throw std::exception(("- Component item " + std::to_string(currentItem) + " has no recipe.").c_str());

			// Has a recipe so recurse down
			auto currentRecipe = recipes.at(currentItem);
			float currentRelativeAssemblerCount = currentRelativeRate / (currentRecipe.rate * currentRecipe.quantity);
			componentItems.insert(currentItem);
			itemRelativeRates.at(currentItem).relativeAssemblerCount += currentRelativeAssemblerCount;
			for (auto ingredient : currentRecipe.ingredients)
			{
				recipeTraceStack.push({ ingredient.item, ingredient.quantity * currentRelativeRate / currentRecipe.quantity });
			}
		}
	}

	// Check the minimum required assemblers can fit
	// Sanity check then bin packing check
	void checkMinimumSpace()
	{
		// Calculate space required
		int blueprintSpace = problem.binWidth * problem.binHeight;
		int minimumSpace = (int)(componentItems.size()) * 10;

		// Cannot fit minimum grid space
		if (blueprintSpace < minimumSpace)
		{
			throw std::exception(("Cannot fit minimumSpace " + std::to_string(minimumSpace) + " into blueprintSpace " + std::to_string(blueprintSpace)).c_str());
		}

		// TODO: Perform bin packing to check if minimum can fit
	}
};

int main()
{
	// Define global parameters
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	// Define problem definition
	ProblemDefinition problem = ProblemDefinitionFactory::newProblemDefinition()
		->setSize(4, 4)
		->addInputItem(0, 1.0f, 0, 0)
		->addOutputItem(2, 3, 3)
		->finalise();

	// Initialize problem solver and solve
	ProblemSolver solver(recipes);
	solver.solve(problem);
}
