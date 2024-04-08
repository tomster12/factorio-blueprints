#pragma once

#include <vector>
#include <fstream>

class Logger
{
public:
	Logger(std::string prefix)
		: prefix(prefix)
	{}

	void log(std::vector<float> row)
	{
		data.push_back(row);
	}

	void save(std::string filename, std::vector<std::string> headers)
	{
		std::ofstream file(prefix + filename + ".csv");

		for (size_t i = 0; i < headers.size(); i++)
		{
			file << headers[i];
			if (i < headers.size() - 1) file << ",";
		}
		file << std::endl;

		for (const auto& row : data)
		{
			for (size_t i = 0; i < row.size(); i++)
			{
				file << row[i];
				if (i < row.size() - 1) file << ",";
			}
			file << std::endl;
		}

		file.close();
	}

private:
	std::string prefix;
	std::vector<std::vector<float>> data;
};
