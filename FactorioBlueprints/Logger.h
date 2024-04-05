#pragma once

#include <vector>
#include <fstream>

class Logger
{
public:
	void log(std::vector<float> row)
	{
		data.push_back(row);
	}

	void save(std::string filename, std::vector<std::string> headers)
	{
		std::ofstream file(filename);

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
	std::vector<std::vector<float>> data;
};
