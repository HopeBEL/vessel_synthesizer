#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vessel_synthesis/binarytree.h>

TEST(binary_tree, create_node)
{
    vs::binary_tree<> tree;
    auto& root = tree.create_root();

    /*=======================================================*/
    auto& child_0 = tree.create_node(root);
    auto& child_1 = tree.create_node(root);

    EXPECT_EQ(root.id(), tree.get_root().id());
    EXPECT_EQ(root.children()[0], child_0.id());
    EXPECT_EQ(root.children()[1], child_1.id());
    EXPECT_EQ(tree.size(), 3);
    /*=======================================================*/

    /*=======================================================*/
    auto& child_2 = tree.create_node(child_0);
    auto& child_3 = tree.create_node(child_1);
    auto& child_4 = tree.create_node(child_1);

    EXPECT_EQ(child_0.children()[0], child_2.id());
    EXPECT_EQ(child_1.children()[0], child_3.id());
    EXPECT_EQ(child_1.children()[1], child_4.id());
    EXPECT_EQ(tree.size(), 6);
    /*=======================================================*/

    /*=======================================================*/
    auto& child_5 = tree.create_node(child_3);
    auto& child_6 = tree.create_node(child_3);

    EXPECT_EQ(child_3.children()[0], child_5.id());
    EXPECT_EQ(child_3.children()[1], child_6.id());
    EXPECT_EQ(tree.size(), 8);
    /*=======================================================*/
}

TEST(binary_tree, delete_node)
{
    vs::binary_tree<> tree;
    auto& root = tree.create_root();

    /*=======================================================*/
    auto& child_0 = tree.create_node(root);
    auto& child_1 = tree.create_node(root);
    auto& child_2 = tree.create_node(child_0);
    auto& child_3 = tree.create_node(child_1);
    auto& child_4 = tree.create_node(child_1);
    tree.create_node(child_3);
    tree.create_node(child_3);
    /*=======================================================*/

    /*=======================================================*/
    tree.delete_node(child_2);

    EXPECT_EQ(child_0.children()[0], vs::not_a_node);
    EXPECT_EQ(child_0.children()[1], vs::not_a_node);
    EXPECT_EQ(tree.size(), 7);
    /*=======================================================*/

    /*=======================================================*/
    tree.delete_node(child_3);

    EXPECT_EQ(child_1.children()[0], vs::not_a_node);
    EXPECT_NE(child_1.children()[1], vs::not_a_node);
    EXPECT_EQ(tree.size(), 4);
    /*=======================================================*/

    /*=======================================================*/
    tree.delete_node(child_4);

    EXPECT_EQ(child_1.children()[0], vs::not_a_node);
    EXPECT_EQ(child_1.children()[1], vs::not_a_node);
    EXPECT_EQ(tree.size(), 3);
    /*=======================================================*/
}

TEST(binary_tree, breadth_first)
{
    vs::binary_tree<> tree;
    auto& root = tree.create_root();
    auto& child_01 = tree.create_node(root);
    auto& child_02 = tree.create_node(root);
    auto& child_03 = tree.create_node(child_02);
    auto& child_04 = tree.create_node(child_03);
    auto& child_05 = tree.create_node(child_03);
    auto& child_06 = tree.create_node(child_04);
    auto& child_07 = tree.create_node(child_05);
    auto& child_08 = tree.create_node(child_01);
    auto& child_09 = tree.create_node(child_01);

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.breadth_first([&](auto& node){ traversalList.emplace_back(node.id()); });
        EXPECT_THAT(traversalList, testing::ElementsAre(root.id(), child_01.id(), child_02.id(), child_08.id(), child_09.id(),
                                                        child_03.id(), child_04.id(), child_05.id(), child_06.id(), child_07.id()));
    }
    /*=======================================================*/
}

TEST(binary_tree, depth_first)
{
    vs::binary_tree<> tree;
    auto& root = tree.create_root();
    auto& child_01 = tree.create_node(root);
    auto& child_02 = tree.create_node(root);
    auto& child_03 = tree.create_node(child_02);
    auto& child_04 = tree.create_node(child_03);
    auto& child_05 = tree.create_node(child_03);
    auto& child_06 = tree.create_node(child_04);
    auto& child_07 = tree.create_node(child_05);
    auto& child_08 = tree.create_node(child_01);
    auto& child_09 = tree.create_node(child_01);

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.depth_first([&](auto& node){ traversalList.emplace_back(node.id()); });
        EXPECT_THAT(traversalList, testing::ElementsAre(root.id(), child_01.id(), child_08.id(), child_09.id(), child_02.id(),
                                                        child_03.id(), child_04.id(), child_06.id(), child_05.id(), child_07.id()));
    }
    /*=======================================================*/
}

TEST(binary_tree, to_root)
{
    vs::binary_tree<> tree;
    auto& root = tree.create_root();
    auto& child_01 = tree.create_node(root);
    auto& child_02 = tree.create_node(root);
    auto& child_03 = tree.create_node(child_02);
    auto& child_04 = tree.create_node(child_03);
    auto& child_05 = tree.create_node(child_03);
    auto& child_06 = tree.create_node(child_04);
    auto& child_07 = tree.create_node(child_05);
    auto& child_08 = tree.create_node(child_01);
    auto& child_09 = tree.create_node(child_01);

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.to_root([&](auto& node){ traversalList.emplace_back(node.id()); }, child_06);
        EXPECT_THAT(traversalList, testing::ElementsAre(child_06.id(), child_04.id(), child_03.id(), child_02.id(), root.id()));
    }
    /*=======================================================*/

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.to_root([&](auto& node){ traversalList.emplace_back(node.id()); }, child_08);
        EXPECT_THAT(traversalList, testing::ElementsAre(child_08.id(), child_01.id(), root.id()));
    }
    /*=======================================================*/

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.to_root([&](auto& node){ traversalList.emplace_back(node.id()); }, child_09);
        EXPECT_THAT(traversalList, testing::ElementsAre(child_09.id(), child_01.id(), root.id()));
    }
    /*=======================================================*/

    /*=======================================================*/
    {
        std::list<vs::node_id> traversalList;
        tree.to_root([&](auto& node){ traversalList.emplace_back(node.id()); }, child_07);
        EXPECT_THAT(traversalList, testing::ElementsAre(child_07.id(), child_05.id(), child_03.id(), child_02.id(), root.id()));
    }
    /*=======================================================*/
}

TEST(binary_tree, custom_data)
{
    struct _Data { int a = 0; double b = 123.0; bool c = false; };

    vs::binary_tree<_Data> tree;
    auto& root = tree.create_root(42, 9.9, false);
    auto& child_01 = tree.create_node(root, 123, 0.0, true);
    auto& child_02 = tree.create_node(child_01);
    auto& child_03 = tree.create_node(child_01, 1, 2.0, false);

    /*=======================================================*/
    EXPECT_EQ(root.data().a, 42);
    EXPECT_EQ(root.data().b, 9.9);
    EXPECT_EQ(root.data().c, false);

    EXPECT_EQ(child_01.data().a, 123);
    EXPECT_EQ(child_01.data().b, 0.0);
    EXPECT_EQ(child_01.data().c, true);

    EXPECT_EQ(child_02.data().a, 0);
    EXPECT_EQ(child_02.data().b, 123.0);
    EXPECT_EQ(child_02.data().c, false);

    EXPECT_EQ(child_03.data().a, 1);
    EXPECT_EQ(child_03.data().b, 2.0);
    EXPECT_EQ(child_03.data().c, false);
    /*=======================================================*/

}



#include <vessel_synthesis/domain.h>
#include <vessel_synthesis/synthesizer.h>
TEST(synthesis, test)
{
    vs::domain_sphere sphere({0.0, 0.0, 0.0}, 0.5);

    vs::synthesizer synth(sphere);
    synth.create_root(vs::system::arterial, {0.49, 0.0, 0.0});
    synth.get_settings().scale(1.5f);
    synth.run();

    auto forest = synth.get_forest(vs::system::arterial);
    forest.delete_if([](auto& t, auto& node) { return node.data().m_radius < 0.004; });

    vs::synthesizer synth2(sphere);
    synth2.set_forest(vs::system::arterial, forest);
    synth.get_settings().scale(1.5f);
    synth2.run();
}
