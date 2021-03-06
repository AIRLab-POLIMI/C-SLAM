cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
project(c_fuzzy)

# If catkin is not found, just compile the executable without ROS goodies
if(catkin_FOUND)
	find_package(Boost REQUIRED COMPONENTS program_options)
    add_message_files(FILES
                      ClassificationOutput.msg  
                      DefuzzyfiedOutput.msg  
                      InputObject.msg  
                      InputVariable.msg  
                      ObjectClassification.msg  
                      SimbolicOutput.msg)
    add_service_files(FILES
    				  Graph.srv 
                      Reasoning.srv 
                      Classification.srv)
    generate_messages()
	catkin_package(CATKIN_DEPENDS message_runtime)
	include_directories(include 
	                    include/lib_fuzzy 
	                    include/lib_tree_classifier 
	                    ${catkin_INCLUDE_DIRS})
else()
	include_directories(${PROJECT_SOURCE_DIR}/include 
	                    ${PROJECT_SOURCE_DIR}/include/lib_fuzzy 
	                    ${PROJECT_SOURCE_DIR}/include/lib_tree_classifier)
endif()


#search for Bison and Flex packages
find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

#Set c++11 flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set the folder of lib_fuzzy and lib_tree_classifier
set(LIB_FUZZY_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/lib_fuzzy)
set(LIB_TREE_CLASSIFIER_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/lib_tree_classifier)


#generate parsers and lexers from flex and Bison sources
bison_target(FuzzyParser 
             ${LIB_FUZZY_SOURCE_DIR}/FuzzyParser.y 
             ${LIB_FUZZY_SOURCE_DIR}/FuzzyParser.tab.cpp 
             COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/lib_fuzzy/FuzzyParser.tab.h")
flex_target(FuzzyScanner 
            ${LIB_FUZZY_SOURCE_DIR}/FuzzyScanner.l 
            ${LIB_FUZZY_SOURCE_DIR}/FuzzyScanner.cpp)

bison_target(TreeClassifierParser 
             ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierParser.y 
             ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierParser.tab.cpp 
             COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/include/lib_tree_classifier/TreeClassifierParser.tab.h")
flex_target(TreeClassifierScanner 
            ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierScanner.l 
            ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierScanner.cpp)

#move generated files into include directory
add_custom_target(HeadersFuzzy
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_FUZZY_SOURCE_DIR}/stack.hh 
		                                 ${PROJECT_SOURCE_DIR}/include/lib_fuzzy/stack.hh
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_FUZZY_SOURCE_DIR}/position.hh 
		                                 ${PROJECT_SOURCE_DIR}/include/lib_fuzzy/position.hh
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_FUZZY_SOURCE_DIR}/location.hh 
                                         ${PROJECT_SOURCE_DIR}/include/lib_fuzzy/location.hh
		DEPENDS ${BISON_FuzzyParser_OUTPUTS})
		
		
add_custom_target(HeadersTreeClassifier
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/stack.hh ${PROJECT_SOURCE_DIR}/include/lib_tree_classifier/stack.hh
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/position.hh ${PROJECT_SOURCE_DIR}/include/lib_tree_classifier/position.hh
		COMMAND ${CMAKE_COMMAND} -E copy ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/location.hh ${PROJECT_SOURCE_DIR}/include/lib_tree_classifier/location.hh
		DEPENDS ${BISON_TreeClassifierParser_OUTPUTS})

#add dependencies between parser and scanner (.l e .y)
add_flex_bison_dependency(FuzzyScanner FuzzyParser)
add_flex_bison_dependency(TreeClassifierScanner TreeClassifierParser)


#build the libraries
add_library(fuzzy STATIC 
			${LIB_FUZZY_SOURCE_DIR}/FuzzyBuilder.cpp 
			${LIB_FUZZY_SOURCE_DIR}/FuzzyVariableEngine.cpp
			${LIB_FUZZY_SOURCE_DIR}/FuzzyMFEngine.cpp
			${LIB_FUZZY_SOURCE_DIR}/FuzzyPredicateEngine.cpp
			${LIB_FUZZY_SOURCE_DIR}/FuzzyKnowledgeBase.cpp 
			${LIB_FUZZY_SOURCE_DIR}/FuzzyMF.cpp  
			${LIB_FUZZY_SOURCE_DIR}/FuzzyOperator.cpp
			${LIB_FUZZY_SOURCE_DIR}/FuzzyReasoner.cpp  
			${LIB_FUZZY_SOURCE_DIR}/FuzzyRule.cpp
			${LIB_FUZZY_SOURCE_DIR}/ReasoningData.cpp
			${LIB_FUZZY_SOURCE_DIR}/VariableMask.cpp 
			${FLEX_FuzzyScanner_OUTPUTS}
			${BISON_FuzzyParser_OUTPUTS})
			
add_library(tree_classifier STATIC 
            ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/ClassifierReasoner.cpp
            ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/VariableGenerator.cpp 
            ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/RuleBuilder.cpp 
			${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierBuilder.cpp 
			${LIB_TREE_CLASSIFIER_SOURCE_DIR}/TreeClassifierScanner.cpp
			${LIB_TREE_CLASSIFIER_SOURCE_DIR}/DependencyGraph.cpp
			${LIB_TREE_CLASSIFIER_SOURCE_DIR}/ReasoningGraph.cpp
			${LIB_TREE_CLASSIFIER_SOURCE_DIR}/FuzzyClassifier.cpp
			${FLEX_TreeClassifierScanner_OUTPUTS}
			${BISON_TreeClassifierParser_OUTPUTS})
			

#remember to move the headers into include!
add_dependencies(fuzzy HeadersFuzzy)
add_dependencies(fuzzy HeadersTreeClassifier)

#build test applications
add_executable(test_reasoner src/testReasoner.cpp)
add_executable(test_classifier src/testClassifier.cpp)

target_link_libraries(test_reasoner fuzzy)
target_link_libraries(test_classifier tree_classifier fuzzy)

#clean all remaining headers
add_custom_command(TARGET fuzzy POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E echo "cleaning *.hh files autogenerated in src/lib_fuzzy"
		COMMAND ${CMAKE_COMMAND} -E remove ${LIB_FUZZY_SOURCE_DIR}/stack.hh 
		                                   ${LIB_FUZZY_SOURCE_DIR}/position.hh 
		                                   ${LIB_FUZZY_SOURCE_DIR}/location.hh)
add_custom_command(TARGET tree_classifier POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E echo "cleaning *.hh files autogenerated in src/lib_tree_classifier"                       
		COMMAND ${CMAKE_COMMAND} -E remove ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/stack.hh 
		                                   ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/position.hh 
		                                   ${LIB_TREE_CLASSIFIER_SOURCE_DIR}/location.hh)
		
		
#now build the ros node
if(catkin_FOUND)

add_executable(${PROJECT_NAME}_reasoner 
               src/main.cpp 
               src/ReasonerServiceHandler.cpp
               src/ClassifierServiceHandler.cpp
               src/CommandLineParser.cpp)
               
target_link_libraries(${PROJECT_NAME}_reasoner 
                      fuzzy 
                      tree_classifier 
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

#add dependency to generated messages
add_dependencies(${PROJECT_NAME}_reasoner ${PROJECT_NAME}_generate_messages_cpp)

endif()
