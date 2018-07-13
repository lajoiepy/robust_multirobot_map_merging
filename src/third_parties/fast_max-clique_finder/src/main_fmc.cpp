/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*   Description:  a library for finding the maximum clique of a graph     		   */                                                   
/*                                                                           		   */
/*                                                                           		   */
/*   Authors: Bharath Pattabiraman and Md. Mostofa Ali Patwary               		   */
/*            EECS Department, Northwestern University                       		   */
/*            email: {bpa342,mpatwary}@eecs.northwestern.edu                 		   */
/*                                                                           		   */
/*   Copyright, 2014, Northwestern University			             		   */
/*   See COPYRIGHT notice in top-level directory.                            		   */
/*                                                                           		   */
/*   Please site the following publication if you use this package:           		   */
/*   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2, 	   	   */
/*   Wei-keng Liao, and Alok Choudhary.	 					   	   */
/*   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with           	   */
/*   Applications to Overlapping Community Detection"				  	   */
/*   http://arxiv.org/abs/1411.7460 		 					   */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "findClique.h"

int iMaxDegree, iMaxClique;

int main(int argc, char *argv[])
{
	int opt, algType=0;
	string strDir, strInput, strOutput;
	int print_clique = 0;

	while ((opt=getopt(argc,argv,"t:o:l:p"))!= EOF)
	{
		switch (opt)
		{
			case 't':
				algType = atoi(optarg);
				break;
			case 'l':
            iMaxClique = atoi(optarg);
            break;
         case 'p':
            print_clique = 1;
            break;
			case '?':
				usage(argv[0]);
				break;
			default:
				usage(argv[0]);
				break;
		}
	}

	if (argc - optind != 1) {
		usage(argv[0]);
		exit(0);
	}

	strInput = argv[optind];

	if(!fexists(strInput.c_str()))
	{
		usage(argv[0]);
		exit(0);
	}
	FILE* fpInputFileNames = fopen(strInput.c_str(), "r+t");

	if(fpInputFileNames == NULL)
	{
		usage(argv[0]);
		exit(0);
	}

	cout << "\n\nFile Name ------------------------ " << strInput.c_str() << endl;
	if(!fexists(strInput.c_str()) )
	{
		cout << "File not found!" << endl;
		return 1;
	}

	CGraphIO gio;
	gio.readGraph(strInput);

	iMaxDegree = gio.GetMaximumVertexDegree();
	cout << "Vertices " << gio.GetVertexCount() << " Edges " << gio.GetEdgeCount() << " Max degree " << iMaxDegree << endl;

	if( iMaxClique > gio.GetVertexCount() || iMaxClique < 0 )
	{
		cout << "ERROR! Lower bound should be in [0," << gio.GetVertexCount()<< "]" << endl;
		return 1;
	}

	double seconds = wtime();
   	vector <int> max_clique_data;

	switch (algType)
	{
		case 0:
			//Exact algorithm with lower bound.
			cout << "Running Exact Algorithm ... " << endl;
			iMaxClique = maxClique(gio, iMaxClique, max_clique_data);
			break;
		case 1:
			//Heuristic
			cout << "Running Heuristic ... " << endl;
			iMaxClique = maxCliqueHeu(gio);
			break;
		default:
			//Exact algorithm with lower bound 0 by default
			iMaxClique = maxClique(gio, 0, max_clique_data);
			cout << "Running Exact Algorithm by default ... " << endl;
			break;
	}

	seconds = wtime() - seconds;

	cout << "Max clique Size : " << iMaxClique << endl;
	fclose(fpInputFileNames);

   if(print_clique == 1 && algType!=1)
      print_max_clique(max_clique_data);

	cout << "Time taken : " << seconds << " SEC" << endl;			

   max_clique_data.clear();

	return 0;
}
