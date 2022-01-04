#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

struct double_node{
	double value;
	double_node *pnext = NULL;
	
	double_node(double a){
		value = a;
	}
}typedef double_node;

struct double_list{
	double_node *head = NULL;
	double_node *tail = NULL;
	int size = 0;
	
	void add_node(double_node *);
	void delete_list();
	void print_list();
	
}typedef double_list;

struct point_node{
	double x;
	double y;
	point_node *pnext = NULL;
	
	point_node(double a, double b){
		x=a;
		y=b;
		}
}typedef point_node;

struct point_list{
	point_node *head = NULL;
	point_node *tail = NULL;
	double x_min, y_min;
	double x_max,y_max;
	int size = 0;
	
	void add_node(point_node *);
	void print_list();
	void delete_list();
	
}typedef point_list;


double sinc(double);
double mod2pi(double);
double rangeSymm(double);
bool check(double, double, double, double, double, double, double, double);
Mat plot_points(point_list *,Mat,Scalar,bool);
void sort(double_list *, point_list *);

#endif