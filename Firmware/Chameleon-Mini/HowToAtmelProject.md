How to generate a project/workspace for the Atmel Studio
========================================================

1. New "Hello World" Project using the Wizard:
	- New GCC C Executable Project
	- Call it 'Chameleon-Mini'
	- Prozessor Atxmega128A4U

2. Clone Chameleon-Mini repo to
	my_local_path/Chameleon-Mini
3. Copy the folowing files of the project generated with ATmel Studio:
	- <Atmel Project>\Chameleon-Mini.atsln --> my_local_path/Chameleon-Mini/Chameleon-Mini.atsln
	- <Atmel Project>\Chameleon-Mini\Chameleon-Mini.cproj --> my_local_path/Chameleon-Mini/Firmware/Chameleon-Mini/Chameleon-Mini.cproj
	- <Atmel Project>\Chameleon-Mini\Chameleon-Mini.componentinfo.xml --> my_local_path/Chameleon-Mini/Firmware/Chameleon-Mini/Chameleon-Mini.componentinfo.xml
4. Adapt Patch in Chameleon-Mini.atsln to the Chameleon-Mini.cproj file
	- Example:
		Project("{54F91283-7BC4-4236-8FF9-10F437C3AD48}") = "Chameleon-Mini", "Firmware\Chameleon-Mini\Chameleon-Mini.cproj", "{DCE6C7E3-EE26-4D79-826B-08594B9AD897}"
5. Now Open the project once to check if it appears properly in the project explorer ... and close it again.

6. We need now to modify the Chameleon-Mini.cproj
    The required input for the *.cproj file can be generate by the make target 'export_cproj_list'

    make export_cproj_list
    
    Attention for 6.1, 6.2, 6.3 : There are RELEASE and DEBUG configuration - add it to both
    

    6.1. Add source files into the <ItemGroup></ItemGroup> section (the one which contains main.c by default - remove main.c entry (the complete <Compile> ... </Compile> section). A more detailed description is part of the Makefile output.
    
    6.2. Add defines into the "\<avrgcc.compiler.symbols.DefSymbols\>". A more detailed description is part of the Makefile output.
    
    6.3. Add include paths in the "\<avrgcc.compiler.directories.IncludePaths\>". A more detailed description is part of the Makefile output.


7. Modify MemoryAsm.S (remove the .skip line)

	--- a/Firmware/Chameleon-Mini/MemoryAsm.S
	
	+++ b/Firmware/Chameleon-Mini/MemoryAsm.S
	
	@@ -222,4 +222,4 @@ FlashCommonSPM:
	
	/* Reserve memory for data */
	
	.section .flashdata
	
	.align 1
	
	-.skip MEMORY_SIZE, MEMORY_INIT_VALUE
	
	+// .skip MEMORY_SIZE, MEMORY_INIT_VALUE
	
