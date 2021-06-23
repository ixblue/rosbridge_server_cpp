#include <QCoreApplication>
#include <QDebug>

#include <librosqt/QRosCallBackQueue.h>
#include <std_msgs/String.h>

#include "ROSNode.h"
#include "WSClient.h"
#include "nlohmann/json.hpp"

//#include "CustomQtestMain.h"
#include "BridgeTester.h"

class MockWSClient : public WSClient
{
    Q_OBJECT
public:
    MockWSClient() : WSClient{nullptr} {}

    void sendMsg(const QString& msg) override { m_lastSentTextMsg = msg; }
    void sendBinaryMsg(const QByteArray& msg) override { m_lastSentBinaryMsg = msg; }
    bool isReady() const override { return true; }
    std::string name() const override { return "mock"; }

    void receivedTextMessage(const QString& msg) { emit onWSMessage(msg); }

    QString m_lastSentTextMsg;
    QByteArray m_lastSentBinaryMsg;
};

void BridgeTester::canSubscribeToATopicAndSendJson()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String"})");

    QVERIFY(client->m_lastSentTextMsg.isEmpty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(20);

    QVERIFY(!client->m_lastSentTextMsg.isEmpty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Encode and decode JSON to get comparable strings
    const auto expectedJsonStr =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json.dump();
    const auto jsonStr =
        nlohmann::json::parse(client->m_lastSentTextMsg.toStdString()).dump();
    QCOMPARE(jsonStr, expectedJsonStr);
}

void BridgeTester::canSubscribeToATopicAndSendCBOR()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello", 10, false);

    // Create on heap because will call deleteLater in destructor
    auto client = new MockWSClient();
    ROSNode node;
    connect(client, &MockWSClient::onWSMessage, &node, &ROSNode::onWSMessage);
    // Send JSON to mock socket
    client->receivedTextMessage(
        R"({"op":"subscribe","topic":"/hello","type":"std_msgs/String","compression":"cbor"})");

    QVERIFY(client->m_lastSentTextMsg.isEmpty());
    QVERIFY(client->m_lastSentBinaryMsg.isEmpty());

    // Publish a message on the topic
    const auto str = "hello";
    {
        std_msgs::String msg;
        msg.data = str;
        pub.publish(msg);
    }

    QTest::qWait(20);

    QVERIFY(client->m_lastSentTextMsg.isEmpty());
    QVERIFY(!client->m_lastSentBinaryMsg.isEmpty());

    const auto expectedJson =
        R"({"op":"publish","topic":"/hello","msg":{"data":"hello"}})"_json;
    const auto expectedCborData = nlohmann::json::to_cbor(expectedJson);
    QCOMPARE(client->m_lastSentBinaryMsg,
             QByteArray(reinterpret_cast<const char*>(expectedCborData.data()),
                        expectedCborData.size()));
}

/*!
 * Catkin ne gere que les tests unitaires avec gtest en C++
 * Or ici, nous utilisons QTest
 * Pour generer un fichier xml de resultat qui sera lu par l'integration continue,
 * nous devons passer des arguments a l'executable des tests unitaires.
 * De plus, la commande `catkin run_tests` ne lance que les tests declares avec
 * `catkin_add_gtest` dans le CMakeLists.txt.
 * Le probleme ici c'est que catkin va donc appeler les executables de tests avec
 * l'argument : `--gtest_output=xml:<chemin_du_fichier_xml>`
 * Or QTest ne comprend pas cet argument.
 *
 * A noter que si le dossier de resultats n'existe pas, Gtest le cree et non QTest.
 * Il y a donc un test si le dossier existe et le cree le cas echeant.
 */
int main(int argc, char* argv[])
{
    // Manipulate args to mimic gtest
    std::vector<std::string> newStringArgs;
    newStringArgs.reserve(argc);
    std::copy(argv, argv + argc, std::back_inserter(newStringArgs));
    /*
    for(size_t i = 0; i < static_cast<size_t>(argc); ++i)
    {
        newStringArgs.emplace_back(argv[i]);
    }
    */
    const std::string gtestArg{"--gtest_output=xml:"};
    auto it = std::find_if(newStringArgs.begin(), newStringArgs.end(),
                           [&gtestArg](const auto& elem) {
                               return elem.find(gtestArg) != std::string::npos;
                           });
    if(it != newStringArgs.end())
    {
        const std::string path = it->substr(gtestArg.length());
        newStringArgs.erase(it);

        QFileInfo outputFile{QString::fromStdString(path)};
        if(!QDir().exists(outputFile.absolutePath()))
        {
            QDir().mkdir(outputFile.absolutePath());
        }
        // Select both XML file and stdout outputs
        newStringArgs.push_back("-o");
        newStringArgs.push_back(path + ",xunitxml");
        newStringArgs.push_back("-o");
        newStringArgs.push_back("-,txt");
    }
    std::vector<char*> newArgv;
    newArgv.reserve(newStringArgs.size());
    for(const auto& arg : newStringArgs)
    {
        newArgv.push_back(const_cast<char*>(arg.c_str()));
    }
    int newArgc = static_cast<int>(newArgv.size());

    QCoreApplication app(newArgc, newArgv.data());

    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(newArgc, newArgv.data(), "bridge_tester",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);

    BridgeTester tester;

    return QTest::qExec(&tester, newArgc, newArgv.data());
}

#include "BridgeTester.moc"
