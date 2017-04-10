# lcd117e_nokia
My version of the awesome LCD117E code modified to compile with Hitech C and use a Nokia1100 display


This version was compiled using MPLAB 8.73 and Hitech-C v9.83a for the 16f688 device

I didn't add my project (.mcp) and workspace (.mcw) files to git, so to compile lcd117e_nokia you will have to set up a new project/workspace.

Along with the code in lcd117e_nokia, you will have to get the pic/code/c/include/lcd1100.h and .c files, and include the .c file as a source file in your mplab project.  You will also have to select the 16f688 device under Configure > Select Device... in Mplab.
