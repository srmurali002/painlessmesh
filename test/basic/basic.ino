#include <arduinoUnity.h>

#include <painlessMesh.h>

#ifdef UNITY

void test_findConnection() {
    painlessMesh mesh;
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(1));

    meshConnectionType conn;
    mesh._connections.push_back(conn);
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(1));

    meshConnectionType conn2;
    conn2.nodeId = 1;
    mesh._connections.push_back(conn2);
    TEST_ASSERT(mesh.findConnection(1));

    // Add test for meshConnection with indirect connection
    meshConnectionType conn3;
    conn3.nodeId = 2;
    conn3.subConnections = "[{\"nodeId\":887034362,\"subs\":[{\"nodeId\":43,\"subs\":[]}]}]";
    mesh._connections.push_back(conn3);
    TEST_ASSERT(mesh.findConnection(887034362));

    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(70));
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(88));
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(87));
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(62));
    TEST_ASSERT_EQUAL(NULL, mesh.findConnection(36));
    // 43 should match again
    TEST_ASSERT(mesh.findConnection(43));
}

bool lesserThan(uint32_t lesser, uint32_t than) {
    return
        (int) lesser - (int) than < 0;// Cast to int in case of time rollover
}

void test_comparison() {
    uint32_t uint32_max = 4294967295;
    TEST_ASSERT(lesserThan(uint32_max-1,uint32_max)); 
    TEST_ASSERT(lesserThan(uint32_max/2-1,uint32_max/2)); 
    TEST_ASSERT(lesserThan(uint32_max/2,uint32_max/2+1)); 
    TEST_ASSERT(lesserThan(10,100)); 

    // Overflow
    TEST_ASSERT(lesserThan(uint32_max,0)); 
    TEST_ASSERT(lesserThan(uint32_max,100)); 
}

void test_subConnectionJsonHelper() {
    meshConnectionType node1, node2;
    node1.nodeId = -1; // Max uint value
    node1.subConnections = String("[{\"nodeId\":886599975,\"subs\":[{\"nodeId\":2139268534,\"subs\":[{\"nodeId\":2132113212,\"subs\":[{\"nodeId\":2132046046}]}]}]}]");
    
    node2.nodeId = 886231565;
    node2.subConnections = String("[{\"nodeId\":2132113139,\"subs\":[]},{\"nodeId\":2132111373,\"subs\":[]}]");

    SimpleList<meshConnectionType>  connections;
    connections.push_back(node1);
    connections.push_back(node2);

    painlessMesh mesh;
    TEST_ASSERT(
        String("[{\"nodeId\":4294967295,\"subs\":[{\"nodeId\":886599975,\"subs\":[{\"nodeId\":2139268534,\"subs\":[{\"nodeId\":2132113212,\"subs\":[{\"nodeId\":2132046046}]}]}]}]},{\"nodeId\":886231565,\"subs\":[{\"nodeId\":2132113139,\"subs\":[]},{\"nodeId\":2132111373,\"subs\":[]}]}]") ==
        mesh.subConnectionJsonHelper(connections));

    TEST_ASSERT(String("[{\"nodeId\":886231565,\"subs\":[{\"nodeId\":2132113139,\"subs\":[]},{\"nodeId\":2132111373,\"subs\":[]}]}]") == 
        mesh.subConnectionJsonHelper(connections, node1.nodeId));

    TEST_ASSERT(String("[{\"nodeId\":4294967295,\"subs\":[{\"nodeId\":886599975,\"subs\":[{\"nodeId\":2139268534,\"subs\":[{\"nodeId\":2132113212,\"subs\":[{\"nodeId\":2132046046}]}]}]}]}]") == 
        mesh.subConnectionJsonHelper(connections, node2.nodeId));
}

void setup() {
    UNITY_BEGIN();    // IMPORTANT LINE!
}

void loop() {
    RUN_TEST(test_findConnection);
    RUN_TEST(test_comparison);
    RUN_TEST(test_subConnectionJsonHelper);
    UNITY_END(); // stop unit testing
}
#endif
