#include "controlmodule.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	controlModule w;
	w.show();
	return a.exec();
}
