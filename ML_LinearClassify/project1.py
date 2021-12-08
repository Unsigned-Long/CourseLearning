# coding:utf-8

# import libs from the sklearn package
from sklearn.datasets import load_iris
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report
from sklearn.model_selection import KFold
from sklearn.model_selection import cross_val_score
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn import svm
from sklearn.metrics import confusion_matrix

# import the pyplot from the matplotlib package
import matplotlib.pyplot as plt


def load_data() -> tuple:
    # load the data by using the load_iris function
    iris_data = load_iris()

    # get values
    iris_data_array = iris_data['data']
    iris_label: list = iris_data['target']
    iris_label_names: list = iris_data['target_names']

    # get the features from the data set
    f0 = [elem[0] for elem in iris_data_array]
    f1 = [elem[1] for elem in iris_data_array]
    f2 = [elem[2] for elem in iris_data_array]
    f3 = [elem[3] for elem in iris_data_array]

    return iris_data, f0, f1, f2, f3, iris_label, iris_label_names


def drawer(x: list, y: list, lab: list, n: list, title: str) -> None:
    # draw the 2d-scatter graphic according feature 0 and 1
    x0, y0 = [], []
    x1, y1 = [], []
    x2, y2 = [], []

    for index in range(len(x)):
        if lab[index] == 0:
            x0.append(x[index])
            y0.append(y[index])
        elif lab[index] == 1:
            x1.append(x[index])
            y1.append(y[index])
        elif lab[index] == 2:
            x2.append(x[index])
            y2.append(y[index])

    plt.scatter(x0, y0, marker='+', c='r', label=n[0])
    plt.scatter(x1, y1, marker='+', c='g', label=n[1])
    plt.scatter(x2, y2, marker='+', c='b', label=n[2])

    # some settings
    plt.legend()
    plt.xlabel('Feature[0]')
    plt.ylabel('Feature[1]')
    plt.title(title)

    # show the graphic
    plt.show()


def split_dataset(data: list):
    return train_test_split(data['data'], data['target'], test_size=0.5)


def split_dataset_k_fold(data: list):
    kf = KFold(n_splits=10, shuffle=True)
    data_ary = data['data']
    target = data['target']
    for train, test in kf.split(data_ary, target):
        ''' for each sub data set, learn the model '''
        # split the data set into train set and test set
        x_train = [data_ary[index] for index in train]
        y_train = [target[index] for index in train]
        x_test = [data_ary[index] for index in test]
        y_test = [target[index] for index in test]

        # train the Logistic Regression model
        sub_model = train_model(x_train, y_train)
        print('--------------------\n' + "parameters\n" + '--------------------')
        print('coef_', sub_model.coef_)
        print('intercept_', sub_model.intercept_)
        y_pred = sub_model.predict(x_test)
        con = confusion_matrix(y_test, y_pred)
        print('--------------------\n' +
              "confusion_matrix\n" + '--------------------')
        print(con)
        print('--------------------\n' + "y_pred\n" +
              '--------------------\n', y_pred)
        print('--------------------\n' + "clf_report\n" + '--------------------')
        clf_report(y_test, y_pred)


def train_model(x_train: list, y_train: list):
    ''' three models '''
    # clf = LogisticRegression()
    # clf = svm.SVC(kernel='linear', probability=True)
    clf = LinearDiscriminantAnalysis(solver='svd', store_covariance=True)

    model = clf.fit(x_train, y_train)
    return model


def clf_report(test_y: list, pred_y: list):
    print(classification_report(test_y, pred_y, target_names=['0', '1', '2']))


def run():
    ''' main function '''
    ''' Step[0] : load the iris data '''
    iris_data, feature0, feature1, feature2, feature3, label, name = load_data()
    ''' Step[1] : drawer(feature0, feature1, label, name) '''
    drawer(feature0, feature1, label, name,
           'Scatter Graphics for Feature[0|1]')
    ''' Step[2] : split the data set into train set and test set '''
    for i in range(5):
        x_train, x_test, y_train, y_test = split_dataset(iris_data)
        ''' Step[3] : train the model and test it '''
        model = train_model(x_train, y_train)
        ''' output some logs '''
        print('--------------------\n' + "parameters\n" + '--------------------')
        print('coef_', model.coef_)
        print('intercept_', model.intercept_)
        y_pred = model.predict(x_test)
        con = confusion_matrix(y_test, y_pred)
        print('--------------------\n' +
              "confusion_matrix\n" + '--------------------')
        print(con)
        print('--------------------\n' + "y_pred\n" +
              '--------------------\n', y_pred)
        print('--------------------\n' + "clf_report\n" + '--------------------')
        clf_report(y_test, y_pred)
    ''' Step[4] : k-fold '''
    split_dataset_k_fold(iris_data)
