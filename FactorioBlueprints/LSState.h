#pragma once

#include "ls.h"
#include "ProblemSolver.h"
#include "CBPathfinder.h"
#include "types.h"

class LSState : public ls::State<LSState>
{
public:
	struct InserterInstance
	{
		int item = -1;
		float rate = 0.0f;
		bool isInput = false;
	};

	struct AssemblerInstance
	{
		static const std::vector<Direction> inserterDirections;
		static const std::vector<Coordinate> inserterOffsets;

		int item = -1;
		Coordinate coordinate;
		InserterInstance inserters[12];
	};

public:
	static std::shared_ptr<LSState> createRandom(const ProblemDefinition& problem, const RunConfig& runConfig);
	LSState(const ProblemDefinition& problem, const RunConfig& runConfig, const std::vector<AssemblerInstance>& assemblers);
	float getFitness() override;
	std::vector<std::shared_ptr<LSState>> getNeighbours() override;
	size_t getHash() override;
	bool getValid();
	void print();
	std::vector<float> generateDataLog() override;

private:
	const ProblemDefinition& problem;
	const RunConfig& runConfig;
	std::vector<AssemblerInstance> assemblers;
	bool costCalculated = false;
	bool neighboursCalculated = false;
	bool worldCalculated = false;
	bool isWorldValid = false;
	std::vector<std::shared_ptr<LSState>> neighbours;
	std::vector<std::vector<bool>> blockedGrid;
	std::vector<std::vector<int>> itemGrid;
	std::vector<ItemEndpoint> itemEndpoints;
	float worldCost = 0.0f;
	std::shared_ptr<CBPathfinder> pathfinder;
	float fitness = 0.0f;
	size_t hash = 0;

	void calculateWorld();
	void calculateHash();
};
