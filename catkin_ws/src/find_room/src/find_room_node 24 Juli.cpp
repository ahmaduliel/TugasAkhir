#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <stdio.h>
#include <math.h>

using namespace std;
#define RADS 57.324

typedef struct{
	float x;
	float y;
}Coordinat;

//variabel bacaData
float data[1080];

//variabel hitungCorner
float selisih[1080];
int indexSelisih[1080];
int iSelisih, zMin, zMax;
float a, b, c, d, e;
float sudutA[1080], sudutB[1080], sudutC[1080];
int iFlag;

//variabel tampungCorner
float tampungCorner[100];
float dataCorner[100][300];
int indexTampung[500], iTampung, iCorner;
int indexData[300][350], jumlahData, iData;
int selisihCorner[300];
int indexCorner[300];

//variabel cariTerendah
float minCorner[50];
int indexMin[50];

//variabel seleksiCorner
float sudutSeleksi[300];
int indexSeleksi[300], iSeleksi;

Coordinat titikCorner[50], normalCorner[50];
Coordinat Corner;
float distCorner;
Coordinat createPoint;

float diagonalCorner, sejajarCorner, jarakCorner[10];
int flagCorner[10];
int Pintu[10];
int iJarak=1, Inc[50], iPintu[50][200];
int titikPintu[10], ititikPintu, normalTitikPintu[10];
Coordinat koorPintu[10], normalKoorPintu[10];
int iDoor;
float jarakDoorCorner[50];
int indexCornerPintu, iCornerPintu, flagCornerPintu;
int resultRoom, flagDuaPintu;
float antarCorner, resultSudut;

Coordinat posNow, koorData[1080], refPos;
int refRoom1, refRoom2, refRoom3, refRoom4, refRoom, ambilSudut;

int selisihCornerPintu[5];

void bacaData(const char *namaFile){
	std::ifstream read;
	read.open(namaFile);
	
	for (int i = 0; i < 1080; i++){
		read >> data[i];
	}
	read.close();
}

void hitungCorner(float minSelisih, int rangeSudut, float minSudut){

	for (int i = 0; i < 1079; i++){
		selisih[i] = (data[i + 1] * 100) - (data[i] * 100);
		if ((selisih[i] >= minSelisih) || (selisih[i] <= -minSelisih)){
			//if (data[i] <= 1.25){
				indexSelisih[iSelisih] = i+1;
				iSelisih++;
			//}
		}
	}

	for (int i = rangeSudut; i < (1080 - rangeSudut); i++){

		zMin = i - rangeSudut;
		zMax = i + rangeSudut;

		a = sqrtf((float)((data[zMin] * data[zMin]) + (data[i] * data[i]) - (2 * data[zMin] * data[i] * cosf((float)(rangeSudut * 0.25) * 1 / RADS))));
		d = sqrtf((float)((data[zMax] * data[zMax]) + (data[i] * data[i]) - (2 * data[zMax] * data[i] * cosf((float)(rangeSudut * 0.25) * 1 / RADS))));

		sudutA[i] = RADS * acosf((float)((a*a) + (data[i] * data[i]) - (data[zMin] * data[zMin])) / (2 * a*data[i]));
		sudutB[i] = RADS * acosf((float)((d*d) + (data[i] * data[i]) - (data[zMax] * data[zMax])) / (2 * d*data[i]));
		sudutC[i] = (sudutA[i] + sudutB[i]) - 180;

		if ((i > (indexSelisih[iFlag] - rangeSudut)) && (i < (indexSelisih[iFlag] + rangeSudut))){
			sudutC[i] = 0;
			if (i == indexSelisih[iFlag]){
				iFlag += 1;
			}
		}

		if (sudutC[i] <= -minSudut){
			tampungCorner[iTampung] = sudutC[i];
			indexTampung[iTampung] = i;
			iTampung++;
		}
	}
}

void tampungData(int minIndex){
	for (int i = 0; i < iTampung; i++){
		dataCorner[iCorner][iData] = tampungCorner[i];
		indexData[iCorner][iData] = indexTampung[i];
		iData++;
		selisihCorner[i] = indexTampung[i + 1] - indexTampung[i];
		if (selisihCorner[i] > minIndex || selisihCorner[i] < -minIndex){
			indexCorner[iCorner] = iData;
			iCorner++;
			iData = 0;
		}
	}
}

void cariTerendah(void){
	for (int i = 0; i < iCorner; i++){
		minCorner[i] = 0;
		for (int j = 0; j < indexCorner[i]; j++){
			if (minCorner[i] > dataCorner[i][j]){
				minCorner[i] = dataCorner[i][j];
				indexMin[i] = indexData[i][j];
			}
		}
	}
}
void seleksiCorner(float minRange){
	for (int i = 0; i < iCorner; i++){
		if (data[indexMin[i]] < minRange){
			sudutSeleksi[iSeleksi] = minCorner[i];
			indexSeleksi[iSeleksi] = indexMin[i];
			iSeleksi++;
		}
	}
}

float calcPolarX(float sudut, float jarak){
	float x;

	x = (float)0 + ((jarak)*cosf((float)(sudut * (1 / RADS))));

	return x;
}
float calcPolarY(float sudut, float jarak){
	float y;

	y = (float)0 + ((jarak)*sinf((float)(sudut * (1 / RADS))));

	return y;
}

float distanceFrom(Coordinat awal, Coordinat akhir){
	Coordinat beda;
	float hasil;
	beda.x = akhir.x - awal.x;
	beda.y = akhir.y - awal.y;
	hasil = sqrt((double)((beda.x*beda.x) + (beda.y*beda.y)));

	return hasil;
}

void titikPotong(Coordinat a, Coordinat b, Coordinat c, Coordinat d){
	float Mab, Mcd, Cab, Ccd;

	Mab = (b.y - a.y) / (b.x - a.x);
	Cab = a.y - (Mab * a.x);

	Mcd = (d.y - c.y) / (d.x - c.x);
	Ccd = c.y - (Mcd * c.x);

	createPoint.x = (Ccd - Cab) / (Mab - Mcd);
	createPoint.y = (Mab*createPoint.x) + Cab;

}

float DistDoor(float angleDoor, float angleCorner, Coordinat Door, Coordinat Corner){
	float selisih, jarak;

	selisih = angleDoor - angleCorner;
	if (selisih <= 360 && selisih >= -360){
		jarak = distanceFrom(Door, Corner);
	}
	else jarak = 0;

	return jarak;
}

void JarakCorner(int iCorner, int indexCorner[], float data[]){
	float selisih[3];

	for (int i = 0; i <iCorner; i++){
		titikCorner[i].x = calcPolarX((indexCorner[i] / 4), data[indexCorner[i]] * 100);
		titikCorner[i].y = calcPolarY((indexCorner[i] / 4), data[indexCorner[i]] * 100);
		normalCorner[i].x = calcPolarX((indexCorner[i] / 4)-45, data[indexCorner[i]] * 100);
		normalCorner[i].y = calcPolarY((indexCorner[i] / 4)-45, data[indexCorner[i]] * 100);
	}

	if (iCorner == 2){
		selisih[0] = indexCorner[0] - indexCorner[1];
		if ((selisih[0] <= 500) && (selisih[0] >= -500)){
			flagCorner[0] = 0;
		}
		else {
			flagCorner[0] = 1;
		}
		jarakCorner[0] = distanceFrom(titikCorner[0], titikCorner[1]);
	}
	else if (iCorner > 2){
		selisih[0] = indexCorner[0] - indexCorner[1];
		selisih[1] = indexCorner[1] - indexCorner[2];
		selisih[2] = indexCorner[0] - indexCorner[2];

		//Flag = 0 => Sejajar
		//Flag = 1 => Diagonal
		for (int i=0; i < 3; i++){
			if ((selisih[i] <= 500) && (selisih[i] >= -500)){
				flagCorner[i] = 0;
			}
			else {
				flagCorner[i] = 1;
			}
			if (i == 2){
				jarakCorner[i] = distanceFrom(titikCorner[0], titikCorner[i]);
				//refRoom = 2;
			}
			else{
				jarakCorner[i] = distanceFrom(titikCorner[i], titikCorner[i + 1]);
				ambilSudut = i;
			}
		}
	}
}

void jarakPintu(int iIndex, int index[], int iCorner, int indexCorner[], Coordinat corner[], int minSelisih){
	int selisihIndex, selisihDoor[10], selisihDuaPintu;
	
	if (iSelisih > 3) flagCornerPintu = 1;

	if (flagCornerPintu == 1){
		for (int i = 0; i < iIndex; i++){
			selisihIndex = index[i + 1] - index[i];
			iPintu[iJarak][Inc[iJarak]] = index[i];
			Inc[iJarak]++;
			if (selisihIndex > minSelisih){
				iJarak++;
			}
		}

		if (iJarak == 1){
			ititikPintu = 1;
			titikPintu[0] = iPintu[1][Inc[1] - 1];
		}
		else if (iJarak == 2){
			ititikPintu = 2;
			titikPintu[0] = iPintu[1][Inc[1] - 1];
			titikPintu[1] = iPintu[2][0]-1;
			selisihDuaPintu = titikPintu[1] - titikPintu[0];
			if (selisihDuaPintu > 300 && selisihDuaPintu < 310){
				ititikPintu = 1;
				flagDuaPintu = 1;
				titikPintu[0] = iPintu[2][Inc[2] - 1];
				titikPintu[1] = 0;
			}
		}

		for (int i = 0; i < ititikPintu; i++){
			koorPintu[i].x = calcPolarX((titikPintu[i] / 4), data[titikPintu[i]] * 100);
			koorPintu[i].y = calcPolarY((titikPintu[i] / 4), data[titikPintu[i]] * 100);
			
			normalTitikPintu[i] = titikPintu[i] - 180;
			if (normalTitikPintu[i] < 0){
				normalTitikPintu[i] = normalTitikPintu[i] - 1440;
			}
			normalKoorPintu[i].x = calcPolarX((normalTitikPintu[i] / 4), data[titikPintu[i]] * 100);
			normalKoorPintu[i].y = calcPolarY((normalTitikPintu[i] / 4), data[titikPintu[i]] * 100);
		}

		//for (int i = 0; i < ititikPintu; i++){
		//	for (int j = 0; j < iSeleksi; j++){
		//		selisihDoor[iDoor] = titikPintu[i] - indexCorner[j];
		//		if (selisihDoor[iDoor] <= 300 && selisihDoor[iDoor] >= -300){
		//			indexCornerPintu = indexCorner[j];
		//			iCornerPintu = j;
		//			jarakDoorCorner[iDoor] = distanceFrom(koorPintu[i], corner[j]);
		//		}
		//		else jarakDoorCorner[iDoor] = 0;
		//		iDoor++;
		//	}
		//}
		for (int i = 0; i < ititikPintu; i++){
			for (int j = 0; j < iSeleksi; j++){
				selisihDoor[iDoor] = titikPintu[i] - indexCorner[j];
				if (selisihDoor[iDoor] <= 300 && selisihDoor[iDoor] >= -300){
					indexCornerPintu = indexCorner[j];
					iCornerPintu = j;	
					//flagCornerPintu = 1;
				}
			}
		}
		for (int i = 0; i < ititikPintu; i++){
			jarakDoorCorner[i] = distanceFrom(koorPintu[i], corner[iCornerPintu]);
		}
	}
}

void cariRuang(){
	int selisih;

	if (flagCornerPintu == 1){
		for (int i = 0; i < ititikPintu; i++){
			if (jarakDoorCorner[i] > 43 && jarakDoorCorner[i] < 55){
				resultRoom = 1;
				refRoom=4;
				printf("Door Detected \n");
			}
			else if (jarakDoorCorner[i] > 70 && jarakDoorCorner[i] < 82){
				resultRoom = 1;
				refRoom=4;
				printf("Door Detected \n");
			}
			else if (jarakDoorCorner[i] > 51 && jarakDoorCorner[i] < 63){
				resultRoom = 2;
				refRoom=4;
				printf("index corner = %d\n", indexSeleksi[iCornerPintu]);
				printf("titik pintu = %d\n", titikPintu[0]);
				printf("titik pintu = %d\n", titikPintu[1]);
				printf("Door Detected 2\n");
			}
			else if (flagDuaPintu == 1 && jarakDoorCorner[i] > 20 && jarakDoorCorner[i] < 26){
				resultRoom = 4;
				refRoom=4;
				printf("Door Detected \n");
			}
			else if (jarakDoorCorner[i] > 20 && jarakDoorCorner[i] < 32){
				
				 for (int i = 0; i < iSeleksi; i++){
				 	 selisihCornerPintu[i] = abs(indexSeleksi[i] - indexSeleksi[iCornerPintu]);
					 if(selisihCornerPintu[i] > 50 && selisihCornerPintu[i] < 400){
						//antarCorner = distanceFrom(titikCorner[i], titikCorner[iCornerPintu]);
					 	flagCorner[i] = 0;
					 	resultRoom = 0;
					 } 
					 else if(selisihCornerPintu[i] > 400){
						//antarCorner = distanceFrom(titikCorner[i], titikCorner[iCornerPintu]);
					 	flagCorner[i] = 1;
					 	resultRoom = 0;
					 }
					printf("antar = %d\n",flagCorner[i]);
				 }
				 
				// for (int i = 0; i < iSeleksi; i++){
				// 	selisih = indexSeleksi[i] - indexCornerPintu;
				// 	if (selisih > 200 && selisih < 450){
				// 		antarCorner = distanceFrom(titikCorner[i], titikCorner[iCornerPintu]);
				// 	}
				// }
				// if (antarCorner > 48 && antarCorner < 55){
				// 	resultRoom = 4;
				// 	refRoom=4;
				// 	printf("Door Detected \n");
				// }
				// else {
				// 	resultRoom = 3;
				// 	refRoom=4;
				// 	printf("Door Detected \n");
				// }
			}
			if (resultRoom == 0){
				if (flagCorner[i] == 1){
					if (jarakCorner[i] > 148 && jarakCorner[i] < 160){
						resultRoom = 1;
						printf("Corner Diagonal 1\n");
					}
					else if (jarakCorner[i] > 119 && jarakCorner[i] < 131){
						resultRoom = 2;
						printf("Corner Diagonal 1\n");
					}
					else if (jarakCorner[i] > 106 && jarakCorner[i] < 118){
						resultRoom = 3;
						printf("Corner Diagonal 1\n");
					}
					else if (jarakCorner[i] > 80 && jarakCorner[i] < 92){
						resultRoom = 4;
						printf("Corner Diagonal 1\n");
					}
				}
				else  if (flagCorner[i] == 0){
					if (jarakCorner[i] > 102 && jarakCorner[i] < 105){
						resultRoom = 2;
						refRoom = 1;
						printf("Corner Parallel 1\n");
					}
					else if (jarakCorner[i] > 69 && jarakCorner[i] < 78){
						resultRoom = 4;
						refRoom = 1;
						printf("Corner Parallel 1\n");
					}
					else if (jarakCorner[i] > 49 && jarakCorner[i] < 54){
						resultRoom = 4;
						refRoom = 2;
						printf("Corner Parallel 1\n");
					}
				}
			}
		}
	}
	else {
		if (resultRoom == 0){
			for (int i = 0; i < iSeleksi; i++){
				if (flagCorner[i] == 1){
					if (jarakCorner[i] > 148 && jarakCorner[i] < 160){
						resultRoom = 1;
						printf("Corner Diagonal \n");
					}
					else if (jarakCorner[i] > 119 && jarakCorner[i] < 131){
						resultRoom = 2;
						printf("Corner Diagonal \n");
					}
					else if (jarakCorner[i] > 106 && jarakCorner[i] < 118){
						resultRoom = 3;
						printf("Corner Diagonal \n");
					}
					else if (jarakCorner[i] > 80 && jarakCorner[i] < 92){
						resultRoom = 4;
						printf("Corner Diagonal \n");
					}
				}
				else  if (flagCorner[i] == 0){
					if (jarakCorner[i] > 102 && jarakCorner[i] < 105){
						resultRoom = 2;
						refRoom = 1;
						printf("Corner Parallel \n");
					}
					else if (jarakCorner[i] > 69 && jarakCorner[i] < 78){
						resultRoom = 3;
						refRoom = 1;
						printf("Corner Parallel \n");
					}
					else if (jarakCorner[i] > 49 && jarakCorner[i] < 54){
						resultRoom = 4;
						refRoom = 2;
						printf("Corner Parallel \n");
					}
				}
			}
		}
	}
}

void cariSudut(){
	if (resultRoom == 1){
		if(refRoom==4){
			resultSudut = (float)(titikPintu[0] - 469)/4;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		// else if(refRoom==0){
		// 	resultSudut = (float)(indexSeleksi[0] - 469)/4;
		// }
		// else if(refRoom==1){
		// 	resultSudut = (float)(indexSeleksi[ambilSudut] - 469)/4;
		// }
		// else if(refRoom==2){
		// 	resultSudut = (float)(indexSeleksi[2] - 469)/4;
		// }
		else { printf("Sudut Tidak Ditemukan\n "); }
	}
	else if (resultRoom == 2){
		if(refRoom==4){
			resultSudut = (float)(titikPintu[0] - 266) / 4;
			resultSudut = resultSudut + 30.0;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==0){
			resultSudut = (float)(indexSeleksi[0] - 469)/4;
		}
		else if(refRoom==1){
			resultSudut = (float)(indexSeleksi[ambilSudut] - 681)/4;
			printf("Done\n");
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			else if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==2){
			resultSudut = (float)(indexSeleksi[2] - 469)/4;
		}
		else { printf("Sudut Tidak Ditemukan \n"); }
		
	}
	else if (resultRoom == 3){
		if(refRoom==4){
			resultSudut = (float)(titikPintu[0] - 102) / 4;
			resultSudut = resultSudut + 60.0;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==0){
			resultSudut = (float)(indexSeleksi[0] - 469)/4;
		}
		else if(refRoom==1){
			resultSudut = (float)(indexSeleksi[ambilSudut] - 505)/4;
			resultSudut = resultSudut + 30.0;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==2){
			resultSudut = (float)(indexSeleksi[2] - 469)/4;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else { printf("Sudut Tidak Ditemukan \n"); }
	}
	else if (resultRoom == 4){
		if(refRoom==4){
			resultSudut = (float)(titikPintu[0] - 639) / 4;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==0){
			resultSudut = (float)(indexSeleksi[0] - 469)/4;
		}
		else if(refRoom==1){
			resultSudut = (float)(indexSeleksi[ambilSudut] - 761)/4;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else if(refRoom==2){
			resultSudut = (float)(indexSeleksi[ambilSudut] - 46)/4;
			if (resultSudut < 0){
				resultSudut = resultSudut + 360.0;
			}
			if (resultSudut > 360){
				resultSudut = resultSudut - 360.0;
			}
		}
		else { printf("Sudut Tidak Ditemukan \n");}
	}
}

void findPos(){
	if(refRoom==4){
		posNow.x = -1*normalKoorPintu[0].x;
		posNow.y = -1*normalKoorPintu[0].y;
	}
	else {
		posNow.x = -1*normalCorner[ambilSudut].x;
		posNow.y = -1*normalCorner[ambilSudut].y;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_node");
	ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Rate r(30);

	bacaData("room4_120.txt");
	hitungCorner(2.0, 30, 60.0);
	tampungData(5);
	cariTerendah();
	seleksiCorner(0.9);
	JarakCorner(iSeleksi, indexSeleksi, data);
	jarakPintu(iSelisih, indexSelisih, iSeleksi, indexSeleksi, titikCorner, 200);
	cariRuang();
	cariSudut();
	findPos();
	printf("\nHasil Room : [%d]\n\n",resultRoom);
	//printf("Hasil Sudut : [%.2f]\n\n",resultSudut);
	//printf("RefRoom : %d\n",refRoom);
	//printf("Ambil Sudut : %d\n\n",ambilSudut);
	for(int i=0; i<iSeleksi; i++){
		printf("Corner = %d  %.2f  %.2f\n",indexSeleksi[i], sudutSeleksi[i], data[indexSeleksi[i]]);
	}
	printf("\n");
	for(int i=0; i<iSeleksi; i++){
		printf("Jarak Corner = %.2f\n",jarakCorner[i]);
	}
	printf("\n");
	for (int i = 0; i < ititikPintu; i++){
		printf("Jarak Pintu = %.2f\n",jarakDoorCorner[i]);
	}
	printf("\n");
	for(int i=0; i<iSeleksi; i++){
		printf("Koor Corner = %.2f   %.2f\n",normalCorner[i].x, normalCorner[i].y);
	}
	printf("\n");
	for(int i=0; i<ititikPintu; i++){
		printf("Koor Pintu = %.2f   %.2f\n",normalKoorPintu[i].x, normalKoorPintu[i].y);
	}
	printf("Pos = %.2f   %.2f\n",posNow.x, posNow.y);


while (ros::ok())
    { 
		visualization_msgs::Marker points, goal, obs, path, room;

		points.header.frame_id = goal.header.frame_id = obs.header.frame_id = path.header.frame_id = room.header.frame_id = "/map";
		//points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
		points.ns = goal.ns = obs.ns = path.ns = room.ns = "points_and_lines";
		points.action = goal.action = obs.action = path.action = room.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = goal.pose.orientation.w = obs.pose.orientation.w = path.pose.orientation.w = room.pose.orientation.w =1.0;

		points.id = 0;
		goal.id = 1;
		obs.id = 2;
		path.id = 3;
		room.id = 4;

		points.type = visualization_msgs::Marker::POINTS;
		goal.type = visualization_msgs::Marker::POINTS;
		obs.type = visualization_msgs::Marker::CYLINDER;
		path.type = visualization_msgs::Marker::LINE_STRIP;
		room.type = visualization_msgs::Marker::POINTS;

	//Konfigurasi titik data
		room.scale.x = 0.01;
		room.scale.y = 0.01;
		room.color.r = 1.0f;
		room.color.g = 1.0f;
		room.color.b = 1.0f;
		room.color.a = 1.0;
		for(int i=0;i<1080;i++){
			koorData[i].x = calcPolarX((i / 4)-45, data[i] * 100);
			koorData[i].y = calcPolarY((i / 4)-45, data[i] * 100);
			
			geometry_msgs::Point dt;
			dt.x = koorData[i].y/100;
			dt.y = -1*koorData[i].x/100;
			room.points.push_back(dt);
		}

	//Konfigurasi Titik Corner
		points.scale.x = 0.05;
		points.scale.y = 0.05;
		points.color.g = 1.0f;
		points.color.a = 1.0;

		for (uint32_t i = 0; i < iSeleksi; i++)
		{
			geometry_msgs::Point d;
			d.x = normalCorner[i].y/100;
			d.y = -1*normalCorner[i].x/100;
			d.z = 0;
			points.points.push_back(d);
		}


	//Konfigurasi Titik Corner
		goal.scale.x = 0.05;
		goal.scale.y = 0.05;
		goal.color.b = 1.0f;
		goal.color.a = 1.0;
		if(refRoom==4){
			geometry_msgs::Point d;
			d.x = normalKoorPintu[0].y/100;
			d.y = -1*normalKoorPintu[0].x/100;
			d.z = 0;
			goal.points.push_back(d);
		}
		else if(refRoom!=4){
			geometry_msgs::Point d;
			d.x = normalCorner[ambilSudut].y/100;
			d.y = -1*normalCorner[ambilSudut].x/100;
			d.z = 0;
			goal.points.push_back(d);
		}
		marker_pub.publish(room);
		marker_pub.publish(points);
		marker_pub.publish(goal);
	}

}
