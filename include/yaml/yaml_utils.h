#define RYML_SINGLE_HDR_DEFINE_NOW
#include "yaml/yaml.h"
#include <unordered_set>
#include <iostream>

#ifndef YAML_UTILS_H 
#define YAML_UTILS_H 

namespace bipedlab
{
namespace yaml_utils
{

std::string getFileContents(const char *filename)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    if (!in) {
        std::cerr << "could not open " << filename << std::endl;
        exit(1);
    }
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

void showProperties(const ryml::NodeRef& root)
{
    std::cout << "tree children: " << root.num_children() << std::endl;
    std::cout << "tree siblings: " << root.num_siblings() << std::endl;
}

void showTree(ryml::NodeRef const& root)
{
    for (ryml::NodeRef const& child : root.children()) {
        std::cout << "- key: " << child.key() << std::endl;
        if (!child.is_seq())
        {
            std::cout << "-- val: " << child.val() << std::endl;
        }
        else
        {
            for (ryml::NodeRef const& n : child.children())
            {
                std::cout << "--- val: " << n.val() << std::endl;
            }
        }
    }
}

std::string 
convertToStr(const c4::csubstr csstring)
{
    return std::string(csstring.str, csstring.len);
}

template<typename T>
std::vector<T> convertToVector(ryml::NodeRef const& ryml_list)
{
    if (!ryml_list.is_seq())
    {   
        return std::vector<T>(std::stoi(convertToStr(ryml_list.val())));
    }
    else
    {
        std::vector<T> ans;
        for (ryml::NodeRef const& n : ryml_list.children())
        {
            ans.push_back(std::stoi(convertToStr(n.val())));
        }
        return ans;
    }   
}

template<typename T>
std::unordered_set<T> convertToUnorderedSet(ryml::NodeRef const& ryml_list)
{
    // std::cout << "bool: " << ryml_list.is_seq() << std::endl;
    // for (ryml::NodeRef const& n : ryml_list.children())
    // {
    //     std::cout << "--- val: " << n.val() << std::endl;
    // }

    if (!ryml_list.is_seq())
    {   
        return {std::stoi(convertToStr(ryml_list.val()))};
    }
    else
    {
        std::unordered_set<T> ans;
        // ans.insert(ryml_list.begin(), ryml_list.end());
        for (ryml::NodeRef const& n : ryml_list.children())
        {
            ans.insert(std::stoi(convertToStr(n.val())));
            // std::cout << "--- val: " << n.val() << std::endl;
        }
        return ans;
    }   
}

template<>
std::unordered_set<std::string> convertToUnorderedSet<std::string>(ryml::NodeRef const& ryml_list)
{
    // std::cout << "bool: " << ryml_list.is_seq() << std::endl;
    // for (ryml::NodeRef const& n : ryml_list.children())
    // {
    //     std::cout << "--- val: " << n.val() << std::endl;
    // }
    if (!ryml_list.is_seq())
    {   
        return {convertToStr(ryml_list.val())};
    }
    else
    {
        std::unordered_set<std::string> ans;
        // ans.insert(ryml_list.begin(), ryml_list.end());
        for (ryml::NodeRef const& n : ryml_list.children())
        {
            ans.insert(convertToStr(n.val()));
            // std::cout << "--- val: " << n.val() << std::endl;
        }
        return ans;
    }
}

}
}
#endif /* ifndef yaml_utils */
