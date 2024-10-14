#pragma once

#include "ls.h"
#include "ProblemSolver.h"
#include "PlacementPathfinder.h"
#include "types.h"

namespace impl
{
	struct ProblemDefinition;
	struct RunConfig;

	class PlacementConfig
	{
	public:
		const ProblemDefinition& problem;
		const RunConfig& runConfig;

		PlacementConfig(const ProblemDefinition& problem, const RunConfig& runConfig)
			: problem(problem), runConfig(runConfig)
		{}
	};

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
		static PlacementState* createRandom(const PlacementConfig& config);

	public:
		PlacementState(const PlacementConfig& config, const std::vector<AssemblerInstance>& assemblerPlacements);
		~PlacementState();
		float getFitness() override;
		std::vector<PlacementState*> getNeighbours(ls::StateCache<PlacementState>& cache) override;
		void clearNeighbours() override;
		bool getValid();
		void print();

	private:
		const PlacementConfig& config;
		const std::vector<AssemblerInstance> assemblerPlacements;

		bool isFitnessCalculated = false;
		float fitness = 0.0f;
		PlacementPathfinder* pathfinder;
		bool isWorldCalculated = false;
		bool isWorldValid = false;
		float worldCost = 0.0f;
		std::vector<std::vector<bool>> blockedGrid;
		std::vector<std::vector<int>> itemGrid;
		std::vector<ItemEndpoint> itemEndpoints;
		bool areNeighboursCalculated = false;
		std::vector<PlacementState*> neighbours;

		void calculateHash();
		void calculateWorld();
	};
}
