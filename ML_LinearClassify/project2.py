# coding:utf-8

# import libs from the sklearn package
from sklearn.datasets import load_iris
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report
from sklearn.metrics import roc_curve
from sklearn.metrics import auc
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn import svm
from sklearn.model_selection import KFold
from sklearn.metrics import confusion_matrix

import pandas as pd
import numpy as np

# import the pyplot from the matplotlib package
import matplotlib.pyplot as plt


def load_data(filename, f1_index, f2_index):
    set = pd.read_excel(filename)
    x = set.iloc[:, [f1_index, f2_index]]
    y = set.iloc[:, [7]]
    x = np.array(x.values.tolist())
    y = np.array(y.values.tolist())
    y = [elem[0] for elem in y]
    f1 = [elem[0] for elem in x]
    f2 = [elem[1] for elem in x]
    label = y
    return f1, f2, label, x, y


def drawer(x: list, y: list, lab: list, n: list, title: str) -> None:
    # draw the 2d-scatter graphic according feature 0 and 1
    x0, y0 = [], []
    x1, y1 = [], []

    for index in range(len(x)):
        if lab[index] == 0:
            x0.append(x[index])
            y0.append(y[index])
        elif lab[index] == 1:
            x1.append(x[index])
            y1.append(y[index])

    plt.scatter(x0, y0, marker='+', c='r', label=n[0])
    plt.scatter(x1, y1, marker='+', c='g', label=n[1])

    # some settings
    plt.legend()


def clf_report(test_y: list, pred_y: list):
    print(classification_report(test_y, pred_y, target_names=['0', '1']))


def roc_plot(y_true, y_score):
    fpr, tpr, thresholds = roc_curve(y_true, y_score)
    roc_auc = auc(fpr, tpr)
    print('--------------------\n' + "AUC\n" + '--------------------')
    print(roc_auc)
    plt.plot(fpr, tpr, c='r', marker='o', markersize=3, label='The Roc Curve')
    plt.annotate('',
                 xy=(0.18, 0.82), xycoords='data',
                 xytext=(0.4, 0.6), textcoords='data',
                 size=15, va="center", ha="center",
                 bbox=dict(boxstyle="round4", fc="w"),
                 arrowprops=dict(arrowstyle="-|>",
                                 connectionstyle="arc3,rad=-0.2",
                                 fc="w"))
    plt.text(0.4, 0.6, 'AUC['+str(roc_auc)+']',
             bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
    plt.legend()
    plt.grid(ls='--', alpha=0.75)
    plt.xlabel('fpr')
    plt.ylabel('tpr')
    plt.title('The ROC Curve for Feature[2|5] By Regress')
    plt.show()


def plot_descisionBoundary(f1, f2, label, model, type):
    drawer(f1, f2, label, ['Type 1', 'Type 2'],
           'Train Scatter Graphics for Feature[2|5]')
    w = model.coef_
    b = model.intercept_
    xp = np.linspace(np.min(f1), np.max(f1), 100)
    yp = -(w[0, 0] * xp + b) / w[0, 1]
    plt.plot(xp, yp, linestyle='dashdot', c='b')
    plt.annotate("Decision Boundary",
                 xy=(1.0, 18.7), xycoords='data',
                 xytext=(1.0, 29), textcoords='data',
                 size=15, va="center", ha="center",
                 bbox=dict(boxstyle="round4", fc="w"),
                 arrowprops=dict(arrowstyle="-|>",
                                 connectionstyle="arc3,rad=-0.2",
                                 fc="w"))
    plt.xlabel('Feature[2]')
    plt.ylabel('Feature[5]')
    plt.title('Decision Boundary for By Regress On '+type+' set[2|5]')
    plt.show()


def run():
    f1_index = 2
    f2_index = 5
    train_filename = r'./data/class_train.xlsx'
    test_filename = r'./data/class_test.xlsx'
    f1_train, f2_train, label_train, x_train, y_train = load_data(
        train_filename, f1_index, f2_index)
    # drawer(f1_train, f2_train, label_train, ['Type 1', 'Type 2'], 'Train Scatter Graphics for Feature[2|5]')
    f1_test, f2_test, label_test, x_test, y_test = load_data(
        test_filename, f1_index, f2_index)
    # drawer(f1_test, f2_test, label_test, ['Type 1', 'Type 2'], 'Test Scatter Graphics for Feature[2|5]')
    clf = LogisticRegression()
    # clf = svm.SVC(kernel='linear', probability=True)
    # clf = LinearDiscriminantAnalysis(solver='svd', store_covariance=True)
    model = clf.fit(x_train, y_train)
    print('--------------------\n' + "parameters\n" + '--------------------')
    print('coef_', model.coef_)
    print('intercept_', model.intercept_)
    y_pred = model.predict(x_test)
    con = confusion_matrix(y_test, y_pred)
    print('--------------------\n' + "confusion_matrix\n" + '--------------------')
    print(con)
    y_prob = model.predict_proba(x_test)
    # print(y_prob)
    print('--------------------\n' + "y_pred\n" +
          '--------------------\n', y_pred)
    print('--------------------\n' + "clf_report\n" + '--------------------')
    clf_report(y_test, y_pred)
    roc_plot(y_test, y_prob[:, 1])
    plot_descisionBoundary(f1_train, f2_train, label_train, model, 'Train')
    plot_descisionBoundary(f1_test, f2_test, label_test, model, 'Test')
