/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.1.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qchartview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionrun;
    QAction *actionsave;
    QAction *actionclear;
    QAction *actionquit;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGridLayout *gridLayout_4;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QPushButton *btn_run;
    QPushButton *btn_clear;
    QPushButton *btn_save;
    QPushButton *btn_quit;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_8;
    QLineEdit *lineEdit_tanDeg_deg;
    QLineEdit *lineEdit_tanDeg_min;
    QLineEdit *lineEdit_tanDeg_sed;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLineEdit *lineEdit_Y_JD;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QComboBox *comboBox_Dir;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit_K_JD;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_7;
    QLineEdit *lineEdit_Ls;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_X_JD;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_6;
    QLineEdit *lineEdit_R;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLineEdit *lineEdit_corDeg_deg;
    QLineEdit *lineEdit_corDeg_min;
    QLineEdit *lineEdit_corDeg_sed;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_8;
    QRadioButton *radioButton_sp;
    QRadioButton *radioButton_rp;
    QTabWidget *tabWidget_mode;
    QWidget *tab_3;
    QGridLayout *gridLayout_5;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_12;
    QLineEdit *lineEdit_sp_k;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_14;
    QLineEdit *lineEdit_sp_x;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_13;
    QLineEdit *lineEdit_sp_y;
    QWidget *tab_4;
    QGridLayout *gridLayout_7;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_16;
    QLineEdit *lineEdit_rp_ek;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_15;
    QLineEdit *lineEdit_rp_sk;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_17;
    QLineEdit *lineEdit_rp_stride;
    QTabWidget *tabWidget_display;
    QWidget *tab;
    QGridLayout *gridLayout_6;
    QVBoxLayout *verticalLayout;
    QLabel *label_9;
    QTableWidget *tableWidget_detail;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_10;
    QTableWidget *tableWidget_calRes;
    QWidget *tab_2;
    QGridLayout *gridLayout_9;
    QLabel *label_11;
    QChartView *graphicsView;
    QMenuBar *menubar;
    QMenu *menuoperations;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(981, 650);
        actionrun = new QAction(MainWindow);
        actionrun->setObjectName(QString::fromUtf8("actionrun"));
        actionsave = new QAction(MainWindow);
        actionsave->setObjectName(QString::fromUtf8("actionsave"));
        actionclear = new QAction(MainWindow);
        actionclear->setObjectName(QString::fromUtf8("actionclear"));
        actionquit = new QAction(MainWindow);
        actionquit->setObjectName(QString::fromUtf8("actionquit"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        scrollArea = new QScrollArea(centralwidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setMaximumSize(QSize(250, 16777215));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 234, 817));
        gridLayout_4 = new QGridLayout(scrollAreaWidgetContents);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        groupBox_2 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        QFont font;
        font.setFamilies({QString::fromUtf8("Ubuntu Mono")});
        font.setPointSize(13);
        groupBox_2->setFont(font);
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        btn_run = new QPushButton(groupBox_2);
        btn_run->setObjectName(QString::fromUtf8("btn_run"));
        btn_run->setMinimumSize(QSize(0, 40));
        btn_run->setFont(font);

        gridLayout_3->addWidget(btn_run, 0, 0, 1, 1);

        btn_clear = new QPushButton(groupBox_2);
        btn_clear->setObjectName(QString::fromUtf8("btn_clear"));
        btn_clear->setMinimumSize(QSize(0, 40));
        btn_clear->setFont(font);

        gridLayout_3->addWidget(btn_clear, 2, 0, 1, 1);

        btn_save = new QPushButton(groupBox_2);
        btn_save->setObjectName(QString::fromUtf8("btn_save"));
        btn_save->setMinimumSize(QSize(0, 40));
        btn_save->setFont(font);

        gridLayout_3->addWidget(btn_save, 1, 0, 1, 1);

        btn_quit = new QPushButton(groupBox_2);
        btn_quit->setObjectName(QString::fromUtf8("btn_quit"));
        btn_quit->setMinimumSize(QSize(0, 40));
        btn_quit->setFont(font);

        gridLayout_3->addWidget(btn_quit, 3, 0, 1, 1);


        gridLayout_4->addWidget(groupBox_2, 2, 0, 1, 1);

        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setFont(font);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMinimumSize(QSize(60, 0));
        label_8->setMaximumSize(QSize(60, 16777215));
        label_8->setFont(font);
        label_8->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(label_8);

        lineEdit_tanDeg_deg = new QLineEdit(groupBox);
        lineEdit_tanDeg_deg->setObjectName(QString::fromUtf8("lineEdit_tanDeg_deg"));
        lineEdit_tanDeg_deg->setFont(font);
        lineEdit_tanDeg_deg->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(lineEdit_tanDeg_deg);

        lineEdit_tanDeg_min = new QLineEdit(groupBox);
        lineEdit_tanDeg_min->setObjectName(QString::fromUtf8("lineEdit_tanDeg_min"));
        lineEdit_tanDeg_min->setFont(font);
        lineEdit_tanDeg_min->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(lineEdit_tanDeg_min);

        lineEdit_tanDeg_sed = new QLineEdit(groupBox);
        lineEdit_tanDeg_sed->setObjectName(QString::fromUtf8("lineEdit_tanDeg_sed"));
        lineEdit_tanDeg_sed->setFont(font);
        lineEdit_tanDeg_sed->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(lineEdit_tanDeg_sed);


        gridLayout_2->addLayout(horizontalLayout_8, 7, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(60, 0));
        label_3->setMaximumSize(QSize(60, 16777215));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(label_3);

        lineEdit_Y_JD = new QLineEdit(groupBox);
        lineEdit_Y_JD->setObjectName(QString::fromUtf8("lineEdit_Y_JD"));
        lineEdit_Y_JD->setFont(font);
        lineEdit_Y_JD->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(lineEdit_Y_JD);


        gridLayout_2->addLayout(horizontalLayout_3, 2, 0, 1, 1);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMinimumSize(QSize(60, 0));
        label_5->setMaximumSize(QSize(60, 16777215));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignCenter);

        horizontalLayout_5->addWidget(label_5);

        comboBox_Dir = new QComboBox(groupBox);
        comboBox_Dir->setObjectName(QString::fromUtf8("comboBox_Dir"));

        horizontalLayout_5->addWidget(comboBox_Dir);


        gridLayout_2->addLayout(horizontalLayout_5, 5, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(60, 0));
        label->setMaximumSize(QSize(60, 16777215));
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label);

        lineEdit_K_JD = new QLineEdit(groupBox);
        lineEdit_K_JD->setObjectName(QString::fromUtf8("lineEdit_K_JD"));
        lineEdit_K_JD->setFont(font);
        lineEdit_K_JD->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(lineEdit_K_JD);


        gridLayout_2->addLayout(horizontalLayout, 0, 0, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setMinimumSize(QSize(60, 0));
        label_7->setMaximumSize(QSize(60, 16777215));
        label_7->setFont(font);
        label_7->setAlignment(Qt::AlignCenter);

        horizontalLayout_7->addWidget(label_7);

        lineEdit_Ls = new QLineEdit(groupBox);
        lineEdit_Ls->setObjectName(QString::fromUtf8("lineEdit_Ls"));
        lineEdit_Ls->setFont(font);
        lineEdit_Ls->setAlignment(Qt::AlignCenter);

        horizontalLayout_7->addWidget(lineEdit_Ls);


        gridLayout_2->addLayout(horizontalLayout_7, 4, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(60, 0));
        label_2->setMaximumSize(QSize(60, 16777215));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(label_2);

        lineEdit_X_JD = new QLineEdit(groupBox);
        lineEdit_X_JD->setObjectName(QString::fromUtf8("lineEdit_X_JD"));
        lineEdit_X_JD->setFont(font);
        lineEdit_X_JD->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(lineEdit_X_JD);


        gridLayout_2->addLayout(horizontalLayout_2, 1, 0, 1, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setMinimumSize(QSize(60, 0));
        label_6->setMaximumSize(QSize(60, 16777215));
        label_6->setFont(font);
        label_6->setAlignment(Qt::AlignCenter);

        horizontalLayout_6->addWidget(label_6);

        lineEdit_R = new QLineEdit(groupBox);
        lineEdit_R->setObjectName(QString::fromUtf8("lineEdit_R"));
        lineEdit_R->setFont(font);
        lineEdit_R->setAlignment(Qt::AlignCenter);

        horizontalLayout_6->addWidget(lineEdit_R);


        gridLayout_2->addLayout(horizontalLayout_6, 3, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMinimumSize(QSize(60, 0));
        label_4->setMaximumSize(QSize(60, 16777215));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(label_4);

        lineEdit_corDeg_deg = new QLineEdit(groupBox);
        lineEdit_corDeg_deg->setObjectName(QString::fromUtf8("lineEdit_corDeg_deg"));
        lineEdit_corDeg_deg->setFont(font);
        lineEdit_corDeg_deg->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(lineEdit_corDeg_deg);

        lineEdit_corDeg_min = new QLineEdit(groupBox);
        lineEdit_corDeg_min->setObjectName(QString::fromUtf8("lineEdit_corDeg_min"));
        lineEdit_corDeg_min->setFont(font);
        lineEdit_corDeg_min->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(lineEdit_corDeg_min);

        lineEdit_corDeg_sed = new QLineEdit(groupBox);
        lineEdit_corDeg_sed->setObjectName(QString::fromUtf8("lineEdit_corDeg_sed"));
        lineEdit_corDeg_sed->setFont(font);
        lineEdit_corDeg_sed->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(lineEdit_corDeg_sed);


        gridLayout_2->addLayout(horizontalLayout_4, 6, 0, 1, 1);


        gridLayout_4->addWidget(groupBox, 0, 0, 1, 1);

        groupBox_3 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setFont(font);
        gridLayout_8 = new QGridLayout(groupBox_3);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        radioButton_sp = new QRadioButton(groupBox_3);
        radioButton_sp->setObjectName(QString::fromUtf8("radioButton_sp"));
        radioButton_sp->setFont(font);

        gridLayout_8->addWidget(radioButton_sp, 0, 0, 1, 1);

        radioButton_rp = new QRadioButton(groupBox_3);
        radioButton_rp->setObjectName(QString::fromUtf8("radioButton_rp"));
        radioButton_rp->setFont(font);

        gridLayout_8->addWidget(radioButton_rp, 1, 0, 1, 1);

        tabWidget_mode = new QTabWidget(groupBox_3);
        tabWidget_mode->setObjectName(QString::fromUtf8("tabWidget_mode"));
        tabWidget_mode->setFont(font);
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        gridLayout_5 = new QGridLayout(tab_3);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_12 = new QLabel(tab_3);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setMinimumSize(QSize(60, 0));
        label_12->setMaximumSize(QSize(60, 16777215));
        label_12->setFont(font);
        label_12->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(label_12);

        lineEdit_sp_k = new QLineEdit(tab_3);
        lineEdit_sp_k->setObjectName(QString::fromUtf8("lineEdit_sp_k"));
        lineEdit_sp_k->setFont(font);
        lineEdit_sp_k->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(lineEdit_sp_k);


        gridLayout_5->addLayout(horizontalLayout_9, 0, 0, 1, 1);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_14 = new QLabel(tab_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setMinimumSize(QSize(60, 0));
        label_14->setMaximumSize(QSize(60, 16777215));
        label_14->setFont(font);
        label_14->setAlignment(Qt::AlignCenter);

        horizontalLayout_11->addWidget(label_14);

        lineEdit_sp_x = new QLineEdit(tab_3);
        lineEdit_sp_x->setObjectName(QString::fromUtf8("lineEdit_sp_x"));
        lineEdit_sp_x->setFont(font);
        lineEdit_sp_x->setAlignment(Qt::AlignCenter);

        horizontalLayout_11->addWidget(lineEdit_sp_x);


        gridLayout_5->addLayout(horizontalLayout_11, 1, 0, 1, 1);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_13 = new QLabel(tab_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setMinimumSize(QSize(60, 0));
        label_13->setMaximumSize(QSize(60, 16777215));
        label_13->setFont(font);
        label_13->setAlignment(Qt::AlignCenter);

        horizontalLayout_10->addWidget(label_13);

        lineEdit_sp_y = new QLineEdit(tab_3);
        lineEdit_sp_y->setObjectName(QString::fromUtf8("lineEdit_sp_y"));
        lineEdit_sp_y->setFont(font);
        lineEdit_sp_y->setAlignment(Qt::AlignCenter);

        horizontalLayout_10->addWidget(lineEdit_sp_y);


        gridLayout_5->addLayout(horizontalLayout_10, 2, 0, 1, 1);

        tabWidget_mode->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        gridLayout_7 = new QGridLayout(tab_4);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_16 = new QLabel(tab_4);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setMinimumSize(QSize(60, 0));
        label_16->setMaximumSize(QSize(60, 16777215));
        label_16->setFont(font);
        label_16->setAlignment(Qt::AlignCenter);

        horizontalLayout_13->addWidget(label_16);

        lineEdit_rp_ek = new QLineEdit(tab_4);
        lineEdit_rp_ek->setObjectName(QString::fromUtf8("lineEdit_rp_ek"));
        lineEdit_rp_ek->setFont(font);
        lineEdit_rp_ek->setAlignment(Qt::AlignCenter);

        horizontalLayout_13->addWidget(lineEdit_rp_ek);


        gridLayout_7->addLayout(horizontalLayout_13, 1, 0, 1, 1);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_15 = new QLabel(tab_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setMinimumSize(QSize(60, 0));
        label_15->setMaximumSize(QSize(60, 16777215));
        label_15->setFont(font);
        label_15->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(label_15);

        lineEdit_rp_sk = new QLineEdit(tab_4);
        lineEdit_rp_sk->setObjectName(QString::fromUtf8("lineEdit_rp_sk"));
        lineEdit_rp_sk->setFont(font);
        lineEdit_rp_sk->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(lineEdit_rp_sk);


        gridLayout_7->addLayout(horizontalLayout_12, 0, 0, 1, 1);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_17 = new QLabel(tab_4);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setMinimumSize(QSize(60, 0));
        label_17->setMaximumSize(QSize(60, 16777215));
        label_17->setFont(font);
        label_17->setAlignment(Qt::AlignCenter);

        horizontalLayout_14->addWidget(label_17);

        lineEdit_rp_stride = new QLineEdit(tab_4);
        lineEdit_rp_stride->setObjectName(QString::fromUtf8("lineEdit_rp_stride"));
        lineEdit_rp_stride->setFont(font);
        lineEdit_rp_stride->setAlignment(Qt::AlignCenter);

        horizontalLayout_14->addWidget(lineEdit_rp_stride);


        gridLayout_7->addLayout(horizontalLayout_14, 2, 0, 1, 1);

        tabWidget_mode->addTab(tab_4, QString());

        gridLayout_8->addWidget(tabWidget_mode, 2, 0, 1, 1);


        gridLayout_4->addWidget(groupBox_3, 1, 0, 1, 1);

        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout->addWidget(scrollArea, 0, 0, 1, 1);

        tabWidget_display = new QTabWidget(centralwidget);
        tabWidget_display->setObjectName(QString::fromUtf8("tabWidget_display"));
        tabWidget_display->setFont(font);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout_6 = new QGridLayout(tab);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_9 = new QLabel(tab);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setMinimumSize(QSize(0, 40));
        label_9->setFont(font);
        label_9->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label_9);

        tableWidget_detail = new QTableWidget(tab);
        tableWidget_detail->setObjectName(QString::fromUtf8("tableWidget_detail"));

        verticalLayout->addWidget(tableWidget_detail);


        gridLayout_6->addLayout(verticalLayout, 0, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_10 = new QLabel(tab);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setMinimumSize(QSize(0, 40));
        label_10->setFont(font);
        label_10->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_10);

        tableWidget_calRes = new QTableWidget(tab);
        tableWidget_calRes->setObjectName(QString::fromUtf8("tableWidget_calRes"));

        verticalLayout_2->addWidget(tableWidget_calRes);


        gridLayout_6->addLayout(verticalLayout_2, 0, 1, 1, 1);

        tabWidget_display->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        gridLayout_9 = new QGridLayout(tab_2);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setMinimumSize(QSize(0, 40));
        label_11->setFont(font);
        label_11->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_11, 0, 0, 1, 1);

        graphicsView = new QChartView(tab_2);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));

        gridLayout_9->addWidget(graphicsView, 1, 0, 1, 1);

        tabWidget_display->addTab(tab_2, QString());

        gridLayout->addWidget(tabWidget_display, 0, 1, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 981, 22));
        menuoperations = new QMenu(menubar);
        menuoperations->setObjectName(QString::fromUtf8("menuoperations"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuoperations->menuAction());
        menuoperations->addAction(actionrun);
        menuoperations->addAction(actionsave);
        menuoperations->addAction(actionclear);
        menuoperations->addSeparator();
        menuoperations->addAction(actionquit);

        retranslateUi(MainWindow);

        tabWidget_mode->setCurrentIndex(1);
        tabWidget_display->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionrun->setText(QCoreApplication::translate("MainWindow", "run", nullptr));
        actionsave->setText(QCoreApplication::translate("MainWindow", "save", nullptr));
        actionclear->setText(QCoreApplication::translate("MainWindow", "clear", nullptr));
        actionquit->setText(QCoreApplication::translate("MainWindow", "quit", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "Operations", nullptr));
        btn_run->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        btn_clear->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        btn_save->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        btn_quit->setText(QCoreApplication::translate("MainWindow", "Quit", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Init values", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "tanDeg", nullptr));
        lineEdit_tanDeg_deg->setText(QCoreApplication::translate("MainWindow", "273", nullptr));
        lineEdit_tanDeg_min->setText(QCoreApplication::translate("MainWindow", "15", nullptr));
        lineEdit_tanDeg_sed->setText(QCoreApplication::translate("MainWindow", "23.4", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Y_JD", nullptr));
        lineEdit_Y_JD->setText(QCoreApplication::translate("MainWindow", "99482.202", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "corDir", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "K_JD", nullptr));
        lineEdit_K_JD->setText(QCoreApplication::translate("MainWindow", "14906.807", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Ls", nullptr));
        lineEdit_Ls->setText(QCoreApplication::translate("MainWindow", "100", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "X_JD", nullptr));
        lineEdit_X_JD->setText(QCoreApplication::translate("MainWindow", "144534.846", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        lineEdit_R->setText(QCoreApplication::translate("MainWindow", "700", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "corDeg", nullptr));
        lineEdit_corDeg_deg->setText(QCoreApplication::translate("MainWindow", "66", nullptr));
        lineEdit_corDeg_min->setText(QCoreApplication::translate("MainWindow", "1", nullptr));
        lineEdit_corDeg_sed->setText(QCoreApplication::translate("MainWindow", "15", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("MainWindow", "Solve Settings", nullptr));
        radioButton_sp->setText(QCoreApplication::translate("MainWindow", "Single Point", nullptr));
        radioButton_rp->setText(QCoreApplication::translate("MainWindow", "Range Points", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "K", nullptr));
        lineEdit_sp_k->setText(QCoreApplication::translate("MainWindow", "14906.807", nullptr));
        label_14->setText(QCoreApplication::translate("MainWindow", "X", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "Y", nullptr));
        tabWidget_mode->setTabText(tabWidget_mode->indexOf(tab_3), QCoreApplication::translate("MainWindow", "SP", nullptr));
        label_16->setText(QCoreApplication::translate("MainWindow", "end K", nullptr));
        lineEdit_rp_ek->setText(QCoreApplication::translate("MainWindow", "16000", nullptr));
        label_15->setText(QCoreApplication::translate("MainWindow", "start K", nullptr));
        lineEdit_rp_sk->setText(QCoreApplication::translate("MainWindow", "14000", nullptr));
        label_17->setText(QCoreApplication::translate("MainWindow", "stride", nullptr));
        lineEdit_rp_stride->setText(QCoreApplication::translate("MainWindow", "10", nullptr));
        tabWidget_mode->setTabText(tabWidget_mode->indexOf(tab_4), QCoreApplication::translate("MainWindow", "RP", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "Details For the Curve", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "Calculation Result", nullptr));
        tabWidget_display->setTabText(tabWidget_display->indexOf(tab), QCoreApplication::translate("MainWindow", "Log", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "Schematic Diagram Of Setting Out Curve", nullptr));
        tabWidget_display->setTabText(tabWidget_display->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Graphics", nullptr));
        menuoperations->setTitle(QCoreApplication::translate("MainWindow", "main", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
