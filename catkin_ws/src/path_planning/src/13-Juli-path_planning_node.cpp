#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <stdio.h>
#include <math.h>

using namespace std;
#define RADS 57.324
#define PI 3.1415926535
#define MAX_RAND 32767

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

//variabel cari selisih
int iFilter, indexFilter[100], indexLingkaran[100], iLingkaran;
int selisihIndex[100];

//variabel cari lingkaran
Coordinat koorCircle1[150], koorCircle2[150];
int iCircle1, iCircle2;
int indexCircle1[150], indexCircle2[150], minIndexCircle1=0, minIndexCircle2;
int sumIndexCircle1, sumIndexCircle2;
float jarakCircle1[150], jarakCircle2[150], minJarakCircle1=0, minJarakCircle2;
float avgCircle1, avgCircle2;
float centerCircle1, centerCircle2;
Coordinat Circle1, Circle2, Start, Goal, Obs;
int rCircle1 = 2, rCircle2 = 15;
int flagCircle1, flagCircle2;

//variabel GA
float randDN[100];
int flagDN, countRand, tempkromosom[100][20], kromosom[100][20];
int kromosomSeleksi[100][20], kromosomSilang[100][20], kromosomMutasi[100][20];
float hasil[100], hasilDN[100];
float valueDN;
int maxPopulasi, maxKromosom;
Coordinat DistNormal[100];
int jumlahGenerasi;
float fitness[100], sumFitness[100], sumDistance[100], sumProb[100], prob[100];
float fitnessSeleksi[100], fitnessSilang[100], fitnessAkhir[100];
int countPilih;
float max[5][100], maxFitness=0, maxDistance[100];
int maxGenerasi, noMax, kromosomMax[100];
Coordinat point[100], realPoint[100];
float Distance[100][10];
int countFitness = 0, flagFitness, seleksi[100];
Coordinat tempPoint[100];
float findParent, acak,cek[10];
int cacah, induk[100];
Coordinat tempDraw[10];
int flagDraw;
int flagRand;
int j, create=0;
float gradienObs[4], gradienStart[3];
Coordinat batasObs[4], batasStart[3];
int countAman, hasilAman;
float bobotAman[100];
Coordinat tempTP[100];
int aman, findFitness=1;

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
			if (data[i] <= 1.25){
				indexSelisih[iSelisih] = i;
				iSelisih++;
			}
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

void findTarget(int object1, int object2){
	if (object1 == 1){
		Obs = Circle1;
		Goal = Circle2;
	}
	else{
		Goal = Circle1;
		Obs = Circle2;
	}
}

void cariLingkaran(){
	for (int i = 0; i < iSelisih; i++){
		if (indexSelisih[i] >= indexSeleksi[0] && indexSelisih[i] <= indexSeleksi[iSeleksi - 1]){
			indexFilter[iFilter] = indexSelisih[i];
			iFilter++;
		}
	}

	for (int i = 0; i < iFilter; i++){
		selisihIndex[i] = indexFilter[i + 1] - indexFilter[i];
		if (selisihIndex[i] >= 4){
			indexLingkaran[iLingkaran] = i;
			iLingkaran++;
		}
	}
	minJarakCircle1 = 100;
	minIndexCircle1 = 1000;
	for (int i = indexFilter[0]; i <= indexFilter[indexLingkaran[1]]; i++){
		jarakCircle1[iCircle1] = data[i]*100;
		koorCircle1[iCircle1].x = calcPolarX((i / 4), data[i] * 100);
		koorCircle1[iCircle1].y = calcPolarY((i / 4), data[i] * 100);
		indexCircle1[iCircle1] = i;
		sumIndexCircle1 += indexCircle1[iCircle1];
		if (minJarakCircle1 > jarakCircle1[iCircle1]){
			minJarakCircle1 = jarakCircle1[iCircle1];
		}
		if (minIndexCircle1 > indexCircle1[iCircle1]){
			minIndexCircle1 = indexCircle1[iCircle1];
		}
		iCircle1++;
	}

	avgCircle1 = (float)sumIndexCircle1 / iCircle1;
	if (iCircle1 >= 30) { centerCircle1 = minJarakCircle1 + rCircle2; flagCircle1=1; }
	else { centerCircle1 = minJarakCircle1 + rCircle1; flagCircle1=0;}
	Circle1.x = (centerCircle1 * (cosf(((avgCircle1 / 4) - 45) * (1 / RADS))));
	Circle1.y = (centerCircle1 * (sinf(((avgCircle1 / 4) - 45) * (1 / RADS))));

	minJarakCircle2 = 100;
	minIndexCircle2 = 1000;
	for (int i = indexFilter[indexLingkaran[1] + 1]; i <= indexFilter[iFilter - 1]; i++){
		jarakCircle2[iCircle2] = data[i] * 100;
		koorCircle2[iCircle2].x = calcPolarX((i / 4), data[i] * 100);
		koorCircle2[iCircle2].y = calcPolarY((i / 4), data[i] * 100);
		indexCircle2[iCircle2] = i;
		sumIndexCircle2 += indexCircle2[iCircle2];
		if (minJarakCircle2 > jarakCircle2[iCircle2]){
			minJarakCircle2 = jarakCircle2[iCircle2];
		}
		if (minIndexCircle2 > indexCircle2[iCircle2]){
			minIndexCircle2 = indexCircle2[iCircle2];
		}
		iCircle2++;
	}
	avgCircle2 = (float)sumIndexCircle2 / iCircle2;
	if (iCircle2 >= 30) { centerCircle2 = minJarakCircle2 + rCircle2; flagCircle2=1;}
	else { centerCircle2 = minJarakCircle2 + rCircle1; flagCircle2=0;}
	Circle2.x = (centerCircle2 * (cosf(((avgCircle2 / 4) - 45) * (1 / RADS))));
	Circle2.y = (centerCircle2 * (sinf(((avgCircle2 / 4) - 45) * (1 / RADS))));
	findTarget(flagCircle1, flagCircle2);
}

float ln(float x){
	float value;
	value = log10f(x) / log10f(2.71828);
	return value;
}

void createDisNormal(float std){
	float dis, value;
	Start.x = 0; Start.y = 0;
	if (flagDN == 1){
		dis = distanceFrom(Start, Goal);
		for (int i = 0; i < maxKromosom; i ++){
			realPoint[i].y = Start.y + (i*((Goal.y-Start.y) / (maxKromosom-1)));
			realPoint[i].x = Start.x + (((Goal.x - Start.x) * (realPoint[i].y - Start.y)) / (Goal.y - Start.y));
			realPoint[0] = Start;
			realPoint[maxKromosom - 1] = Goal;
			flagRand = 0;
			randDN[i] = ((float)rand() / (RAND_MAX/2))-1.0;
			randDN[0] = 0.999;
			if (randDN[i] < 0) { flagRand = 1; }
			hasil[i] = (std * sqrtf(-2 * (1 / (std*sqrt((float)2 * PI))) *ln(abs(randDN[i]))));
			if (flagRand == 1) { 
				hasil[i] = -1 * hasil[i];
			}
			hasilDN[i] = hasil[i] + realPoint[i].x;
		}
		flagDN = 0;
	}
	for (int i = 1; i < (maxKromosom-1); i++){
		point[i].x = hasilDN[i];
		point[i].y = realPoint[i].y;
		point[0] = Start;
		point[maxKromosom - 1] = Goal;
		// glColor3d(1, 0, 0);
		// solidCircle(point[i].x, point[i].y, 1);
		// glBegin(GL_LINES);
		// glColor3d(1, 0, 0);
		// glVertex3f(start.x, start.y, -100);
		// glVertex3f((goal.x), (goal.y), -100);
		// glEnd();
	}
}

void inisialisasi(void){
	int num;
	for (int i = 0; i < maxPopulasi; i++){
		num = i;
		for (int j = 1; j < maxKromosom - 1; j++){
			kromosom[i][j] = num % 2;
			num /= 2;
			kromosom[i][0] = 1;
			kromosom[i][maxKromosom - 1] = 1;
			tempkromosom[i][j] = kromosom[i][j];
			tempkromosom[i][0] = 1;
			tempkromosom[i][maxKromosom - 1] = 1;
		}
	}
}

int cekAman(Coordinat titik){
	int hasil = 0;
	float jarak = 0;
	
	jarak = sqrt((double)(((Obs.x - titik.x)*(Obs.x - titik.x)) + ((Obs.y - titik.y)*(Obs.y - titik.y))));

	if (jarak < 15.0){
		hasil = 1;
	}

	return hasil;
}

void evaluasi(void){
	for (int i = 0; i < maxPopulasi; i++){
		countAman = 0;
		sumDistance[i] = 0;
		for (int j = 1; j < maxKromosom; j++){
			hasilAman = 0;
			if (kromosom[i][j] == 1){
				tempPoint[j] = point[j];
			}
			else if (kromosom[i][j] == 0){
				tempPoint[j] = realPoint[j];
			}
			hasilAman = cekAman(tempPoint[j]);
			if (hasilAman == 1) countAman+=1;
			tempPoint[0] = Start;
			Distance[i][j] = distanceFrom(tempPoint[j - 1], tempPoint[j]);
			sumDistance[i] += Distance[i][j];
		}
		if (countAman < 1) { bobotAman[i] = 10.0; }
		else { bobotAman[i] = 0; }
		fitness[i] = (1 / sumDistance[i]) * bobotAman[i];
		sumFitness[i] = sumFitness[i - 1] + fitness[i];
		//printf("%.3f \n", fitness[i]);
	}
}

void probFitness(void){
	for (int i = 0; i < maxPopulasi; i++){
		prob[i] = fitness[i] / sumFitness[maxPopulasi-1];
		sumProb[i] = sumProb[i - 1] + prob[i];
	}
}

void prediksi(void){
	int j,l=1;
	findParent = 0;
	for (int i = 0; i < maxPopulasi; i++){
		acak = ((float)rand() / (RAND_MAX));
		for (j = 0; j < maxPopulasi; j++){
			if (acak <= sumProb[j]) {
				break;
			}
		}
		seleksi[i] = j;
	}
}

void changeKromosom(void){
	for (int i = 0; i < maxPopulasi; i++){
		for (int j = 0; j < maxKromosom; j++){
			kromosomSeleksi[i][j] = kromosom[seleksi[i]][j];
		}
		fitnessSeleksi[i] = fitness[seleksi[i]];
	}
}

void silang(float probSilang){
	cacah = 0;
	for (int j = 0; j < 10; j++){
		for (int i = 0; i < maxPopulasi; i++){
			if (random() <= probSilang){
				cacah+=1;
				induk[cacah] = i;
			}
			if (cacah > 2) break;
		}
		if (cacah > 2) break;
	}
	for (int i = 0; i < maxPopulasi; i++){
		//printf("%d ", i);
		for (int j = 0; j < maxKromosom; j++){
			kromosomSilang[induk[1]][j] = kromosomSeleksi[induk[2]][j];
			kromosomSilang[induk[2]][j] = kromosomSeleksi[induk[1]][j];
			fitnessSilang[induk[1]] = fitnessSeleksi[induk[2]];
			fitnessSilang[induk[2]] = fitnessSeleksi[induk[1]];

			kromosomSilang[i][j] = kromosomSeleksi[i][j];
			fitnessSilang[i] = fitnessSeleksi[i];

			kromosom[i][j] = kromosomSilang[i][j];
			fitnessAkhir[i] = fitnessSilang[i];
			//printf("%d ", kromosom[i][j]);
		}
		//printf("%.3f \n", fitnessAkhir[i]);
	}
}

void mutasi(float probMutasi){
	for (int i = 0; i < maxPopulasi; i++){
		for (int j = 0; j < maxKromosom; j++){
			if (random() <= probMutasi){
				if (kromosomMutasi[i][j] == 1){
					kromosomMutasi[i][j] == 0;
				}
				else {
					kromosomMutasi[i][j] == 1;
				}
			}
			kromosom[i][j] = kromosomMutasi[i][j];
		}
	}
}

void historis(int nGenerasi){
	for (int j = 0; j < maxPopulasi; j++){
		if (maxFitness < fitnessAkhir[j]){
			maxFitness = fitnessAkhir[j];
			noMax = j;
			for (int k = 1; k < maxKromosom; k++){
				kromosomMax[0] = 1;
				kromosomMax[k] = kromosom[j][k];
				Distance[j][0] = 0;
				maxDistance[k] = maxDistance[k - 1] + Distance[j][k];
			}
			countPilih++;
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Rate r(30);

	bacaData("Circle_2.txt");
	hitungCorner(3.0, 30, 60.0);
	tampungData(5);
	cariTerendah();
	seleksiCorner(1.25);
	cariLingkaran();
	printf("Flag 1 = %d  Flag 2 = %d\n", flagCircle1, flagCircle2);
    printf("Goal = X : %.2f   Y : %.2f\n", Goal.x, Goal.y);
    printf("Obs = X : %.2f   Y : %.2f\n", Obs.x, Obs.y);
	printf("Center = 1 : %.3f   2 : %.3f\n", centerCircle1, centerCircle2);
	
	maxPopulasi = 64;
	maxKromosom = 8;

	while(maxFitness==0){
		flagDN = 1;
		flagDraw = 0;	
		createDisNormal(50.0);
		inisialisasi();
		evaluasi();
		probFitness();
		prediksi();
		changeKromosom();
		silang(0.6);
		historis(1);
		jumlahGenerasi++;
	}

	printf("Max Fitness = %.3f\n", maxFitness);
	
	for(int i=0; i<maxKromosom; i++){
		printf("Random [%d]= %.3f   %.3f   %.3f   %.3f\n",i , randDN[i], hasil[i], realPoint[i].x, hasilDN[i]);
	}

	for(int i=0; i<maxKromosom; i++){
		printf("%d ", kromosomMax[i]);
	}
	printf("\n");
	for(int i=0; i<maxKromosom; i++){
		printf("Max Distance = %.3f\n", maxDistance[i]);
	}

	while (ros::ok())
    { 
    visualization_msgs::Marker points, text, arah, line, titik, obs;
     points.header.frame_id = text.header.frame_id = arah.header.frame_id = line.header.frame_id = "/map";
     titik.header.frame_id = obs.header.frame_id = "/map";
     //points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
     points.ns = text.ns = arah.ns = line.ns = "points_and_lines";
     titik.ns = obs.ns = "points_and_lines";
     points.action = text.action = arah.action = line.action = visualization_msgs::Marker::ADD;
     titik.action = obs.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = text.pose.orientation.w = arah.pose.orientation.w = line.pose.orientation.w = 1.0;
 	 titik.pose.orientation.w = obs.pose.orientation.w = 1.0;
 
     points.id = 0;
     text.id = 1;
     arah.id = 2;
     line.id = 3;
     titik.id = 4;
     obs.id = 5;

     points.type = visualization_msgs::Marker::POINTS;
     text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
     arah.type = visualization_msgs::Marker::ARROW;
     line.type = visualization_msgs::Marker::LINE_STRIP;
	 titik.type = visualization_msgs::Marker::POINTS;
	 obs.type = visualization_msgs::Marker::POINTS;

     arah.pose.position.x = -0.15;
     arah.pose.position.y = 0.2;
     arah.pose.position.z = 0.0;
     arah.pose.orientation.x = 0.0;
     arah.pose.orientation.y = 0.0;
     arah.pose.orientation.z = 0.0;
     arah.pose.orientation.w = 1.0;

     arah.scale.x = 0.3;
     arah.scale.y = 0.03;
     arah.scale.z = 0.08;

     arah.color.r = 0.0f;
     arah.color.g = 1.0f;
     arah.color.b = 0.0f;
     arah.color.a = 1.0;

      // POINTS markers use x and y scale for width/height respectively
     points.scale.x = 0.03;
     points.scale.y = 0.03;
     
	 titik.scale.x = 0.03;
     titik.scale.y = 0.03;

	 obs.scale.x = 0.03;
     obs.scale.y = 0.03;
	 
     line.scale.x = 0.01;

     text.pose.position.x = 0.0;
     text.pose.position.y = 1.0;
     text.pose.position.z = 0.0;
     text.pose.orientation.x = 0.0;
     text.pose.orientation.y = 0.0;
     text.pose.orientation.z = 0.0;
     text.pose.orientation.w = 1.0;

     text.text = "blablabla";

     text.scale.x = 0.3;
     text.scale.y = 0.3;
     text.scale.z = 0.1;

     text.color.r = 0.0f;
     text.color.g = 1.0f;
     text.color.b = 0.0f;
     text.color.a = 1.0;
     
     // Points are green
     points.color.g = 1.0f;
     points.color.a = 1.0;
 
     line.color.b = 1.0f;
     line.color.a = 1.0;
 
      //for (uint32_t i = 0; i < 3; ++i)
       //{
         geometry_msgs::Point p, q;
         p.x = Goal.y/100;
         p.y = -1*Goal.x/100;
         p.z = 0;
		 points.points.push_back(p);
		 //line.points.push_back(p);

		 p.x = 0;
         p.y = 0;
         p.z = 0;
         points.points.push_back(p);
         //line.points.push_back(p);

		 p.x = Obs.y/100;
         p.y = -1*Obs.x/100;
         p.z = 0;
         points.points.push_back(p);
       //}

       for (uint32_t i = 1; i < maxKromosom-1; i++)
       {
         geometry_msgs::Point d;
         d.x = realPoint[i].y/100;
         d.y = -1*hasilDN[i]/100;
         d.z = 0;
		 points.points.push_back(d);
       }
	   for (j = 0; j < maxKromosom; j++){
			if (kromosomMax[j] == 1){
				tempDraw[j] = point[j];
			}
			else if (kromosomMax[j] == 0){
				tempDraw[j] = realPoint[j];
			}
			//tempDraw[0-1] = Start;
			tempDraw[0] = Start;
			tempDraw[maxKromosom-1] = Goal;
			
			geometry_msgs::Point v;
			v.x = tempDraw[j].y/100;
         	v.y = -1*tempDraw[j].x/100;
         	v.z = 0;
		 	line.points.push_back(v);
		}
 
    //marker_pub.publish(text);
     marker_pub.publish(points);
    //  marker_pub.publish(arah);
      marker_pub.publish(line);
	}
	//ros::spin();
}