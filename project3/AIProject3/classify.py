import pickle
import torch
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.decomposition import PCA
from sklearn.model_selection import train_test_split


def classify():
    data = torch.load("data.pth")
    label = data["label"]
    feature = data["feature"]
    feature = feature.numpy()
    label = label.numpy()
    feature_train, feature_test, label_train, label_test = train_test_split(feature, label, test_size=0.2, random_state=0)
    pca = PCA(n_components=128)
    feature_train = pca.fit_transform(feature_train)
    knn = KNeighborsClassifier(n_neighbors=10)
    knn.fit(feature_train, label_train)
    feature_test = pca.transform(feature_test)
    print(knn.score(feature_test, label_test))
    if knn.score(feature_test, label_test) > 0.95:
        torch.save(knn, "knn.pt")
        torch.save(pca, "pca.pt")

# def verify():
#     model = torch.load('knn.pt')
#     pca = torch.load('pca.pt')
#     data = torch.load("data.pth")
#     label = data["label"][0:100]
#     feature = data["feature"][0:100]
#     feature = feature.numpy()
#     label = label.numpy()
#     feature = pca.transform(feature)
#     print(model.score(feature, label))


if __name__ == "__main__":

    classify()
    # verify()
