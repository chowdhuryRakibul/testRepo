1. Download the workspace

/*************************Example File Directory*****************************/
Dias32FirmwareDevelopment (parent)
|--ChibiOSRepo
|  |--chibios191
|--ChibiOS-Contrib #(git clone https://github.com/ChibiOS/ChibiOS-Contrib.git)
|--FirmwareRepo
|  |--Dias32Workspace
/****************************************************************************/

2. (if not already done) Go to chibios191/ext/ and unzip fatfs-0.xx_patched at the same location
3. Go to C:\ChibiStudio and open "Chibi Studio GCC 7.0"
2. From File > Switch Workspace > Other select the workspace folder that was just downloaded.
   It should also import the projects by default, if not then just go to 
   File>Import>Existing Projects Into Workspace and Browse to the workspace folder to select projects
3. Now Select DIAS-Gen2-testing project, right click and go to properties>Resource>Linked Resources
   and Edit the location of CHIBIOS variable. Link it to chibios191 folder
4. Change the absolute location of CHIBIOS and CHIBIOS_CONTRIB in the Makefile (line 90/91) (if needed)



Note:
.sc files from ".metadata>.plugins>org.eclipse.cdt.make.core" has been removed/.gitignored as this file contains
the paths to include and Eclipse only appends this file. So, when the project is rebuilt at other locations, 
it may create pathNotFound warnings. As a flipside of removing this file, you may not see the includes files in 
your Project Explorer.