
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
struct ComponentItemInfo { int item; };

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
	}

private:
	const std::map<int, Recipe>& recipes;
	ProblemDefinition problem;
	std::vector<ComponentItemInfo> componentItems;

	// Unravel output item recipe to inputs
	// Fails early if an item cannot be crafted from inputs
	void unravelRecipes()
	{
		// Produce a set of input items
		std::set<int> itemInputs;
		for (int i = 0; i < problem.itemInputs.size(); i++) itemInputs.insert(problem.itemInputs[i].item);

		// Setup recipe unravel loop
		componentItems.clear();
		std::stack<ComponentItemInfo> currentComponentItems;
		int itemOutput = problem.itemOutput.item;
		currentComponentItems.push({ itemOutput });
		while (currentComponentItems.size() > 0)
		{
			// Pop current item from queue
			ComponentItemInfo currentComponentItem = currentComponentItems.top();
			std::cout << "Checking item " << currentComponentItem.item << std::endl;
			currentComponentItems.pop();

			// Skip if is an input item
			if (itemInputs.find(currentComponentItem.item) != itemInputs.end())
			{
				std::cout << ": Found input " << std::endl;
				continue;
			}

			// Has no recipe so impossible problem
			if (recipes.find(currentComponentItem.item) == recipes.end())
			{
				throw std::exception(("- Item " + std::to_string(currentComponentItem.item) + " is not an input nor can be crafted.").c_str());
			}

			// Has a recipe so recurse down
			componentItems.push_back(currentComponentItem);
			for (auto ingredient : recipes.at(currentComponentItem.item).ingredients)
			{
				std::cout << ": Found ingredient " << ingredient.item << std::endl;
				currentComponentItems.push({ ingredient.item });
			}
		}

		// Log unique items
		std::cout << std::endl << "Uniques items: ";
		for (ComponentItemInfo componentItem : componentItems) std::cout << componentItem.item << " ";
		std::cout << std::endl;
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

		// Perform bin packing to check if minimum can fit
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
