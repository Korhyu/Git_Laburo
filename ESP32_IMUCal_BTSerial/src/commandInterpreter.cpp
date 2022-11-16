#include "commandInterpreter.h"



struct command commandInterpreter (std::string str)
{
	std::string header = "";
	std::string parameter = "";

	header += str[0];
	header += str[1];
	header += str[2];

	parameter += str[0];
	parameter += str[1];

	if(strcmp(&str[2],"\r") != 0)
	{
		parameter += str[2];

		if(strcmp(&str[3],"\r") != 0)
		{
			parameter += str[3];
		}
		else
		{
			parameter += "\n";
		}
	}
	else
	{
		parameter += "\n";
	}

	struct command cmd;
	
	for(int i=0; i<INFO_NUMBER; i++)
	{
		if (header.compare(cmdInfo[i]) == 0)
			break;
	}

	cmd.parameter = std::stoi( parameter );

	return cmd;
	
}