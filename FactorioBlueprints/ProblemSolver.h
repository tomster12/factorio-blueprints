#pragma once

#include "dataLogger.h"
#include "types.h"

struct ProblemDefinition
{
	struct ItemInput
	{
		int item = -1;
		float rate = 0;
		Coordinate coordinate;
	};

	struct ItemOutput
	{
		int item = -1;
		Coordinate coordinate;
	};

	int blueprintWidth = -1;
	int blueprintHeight = -1;
	std::map<int, Recipe> recipes;
	std::map<int, ItemInput> itemInputs;
	ItemOutput itemOutput;
};

struct RunConfig
{
	struct ItemConfig
	{
		struct InserterRequirement { int count; float rate; };

		int item = -1;
		int assemblerCount = 0;
		InserterRequirement outputInserterRequirement = { 0, 0 };
		std::map<int, InserterRequirement> inputInserterRequirements;
	};

	int outputAssemblerCount = 0;
	std::map<int, ItemConfig> itemConfigs;
};

class ProblemDefinitionFactory
{
public:
	static std::unique_ptr<ProblemDefinitionFactory> create();
	ProblemDefinitionFactory* setRecipes(std::map<int, Recipe> recipes);
	ProblemDefinitionFactory* setSize(int blueprintWidth, int blueprintHeight);
	ProblemDefinitionFactory* addInputItem(int inputItem, float inputRate, int x, int y);
	ProblemDefinitionFactory* addOutputItem(int itemOutput, int x, int y);
	ProblemDefinition finalise();

private:
	ProblemDefinition problemDefinition;
};

class ProblemSolver
{
public:
	static ProblemSolver solve(const ProblemDefinition& problem, std::shared_ptr<DataLogger> dataLogger = nullptr);

private:
	struct ItemInfo
	{
		int item = -1;
		bool isComponent = false;
		float rate = 0.0f;
	};

	const ProblemDefinition& problem;
	std::shared_ptr<DataLogger> dataLogger;
	int componentItemCount = -1;
	int bestRunConfig = -1;
	std::map<int, ItemInfo> baseItemInfos;
	std::map<int, RunConfig> possibleRunConfigs;

	ProblemSolver(const ProblemDefinition& problem, std::shared_ptr<DataLogger> dataLogger = nullptr);
	void solve();
	void unravelRecipes();
	void calculateRunConfigs();
	void performSearch();
};
