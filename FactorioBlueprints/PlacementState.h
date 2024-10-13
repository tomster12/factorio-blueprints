#pragma once

#include "ls.h"
#include "ProblemSolver.h"
#include "PlacementPathfinder.h"
#include "types.h"

namespace impl
{
	class PlacementState : public ls::State<PlacementState>
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
		static std::shared_ptr<PlacementState> createRandom(const ProblemDefinition& problem, const RunConfig& runConfig);
		PlacementState(const ProblemDefinition& problem, const RunConfig& runConfig, const std::vector<AssemblerInstance>& assemblers);
		float getFitness() override;
		std::vector<std::shared_ptr<PlacementState>> getNeighbours() override;
		bool getValid();
		void print();

	private:
		const ProblemDefinition& problem;
		const RunConfig& runConfig;
		std::vector<AssemblerInstance> assemblers;
		bool costCalculated = false;
		bool neighboursCalculated = false;
		bool worldCalculated = false;
		bool isWorldValid = false;
		std::vector<std::shared_ptr<PlacementState>> neighbours;
		std::vector<std::vector<bool>> blockedGrid;
		std::vector<std::vector<int>> itemGrid;
		std::vector<ItemEndpoint> itemEndpoints;
		float worldCost = 0.0f;
		std::shared_ptr<PlacementPathfinder> pathfinder;
		float fitness = 0.0f;

		void calculateWorld();
		void calculateHash();
	};
}
