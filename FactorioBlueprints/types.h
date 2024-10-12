#pragma once

#include <vector>
#include <cstdint>
#include <string>

enum class BeltType : uint8_t { None = 1, Inserter = 2, Conveyor = 4, UndergroundEntrance = 8, Underground = 16, UndergroundExit = 32 };

enum class Direction : uint8_t { N = 1, S = 2, W = 4, E = 8 };

struct Coordinate { int x = 0; int y = 0; };

inline Direction dirOpposite(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return Direction::S;
	case Direction::S: return Direction::N;
	case Direction::W: return Direction::E;
	case Direction::E: return Direction::W;
	}
	return Direction::N;
}

inline Coordinate dirOffset(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return { 0, -1 };
	case Direction::S: return { 0, 1 };
	case Direction::W: return { -1, 0 };
	case Direction::E: return { 1, 0 };
	}
	return { 0, 0 };
}

inline std::string dirString(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return "N";
	case Direction::S: return "S";
	case Direction::W: return "W";
	case Direction::E: return "E";
	}
	return "N";
}

struct Recipe
{
	struct Ingredient
	{
		int item = -1;
		int quantity = 0;
	};

	int quantity = 0;
	float rate = 0;
	std::vector<Ingredient> ingredients;
};

const std::vector<Direction> INSERTER_DIRECTIONS = {
	Direction::N, Direction::N, Direction::N,
	Direction::E, Direction::E, Direction::E,
	Direction::S, Direction::S, Direction::S,
	Direction::W, Direction::W, Direction::W
};

const std::vector<Coordinate> INSERTER_OFFSETS = {
	{ 0, -1 }, { 1, -1 }, { 2, -1 },
	{ 3, 0 }, { 3, 1 }, { 3, 2 },
	{ 2, 3 }, { 1, 3 }, { 0, 3 },
	{ -1, 2 }, { -1, 1 }, { -1, 0 }
};

static const float MAX_INSERTER_RATE = 4.62f;
static const float MAX_CONVEYOR_RATE = 45.0f;
