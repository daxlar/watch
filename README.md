As of nRF Connect SDK 2.5, the way that this repo is going to be used is to perform the following steps:  

1). Install VSCode and the nRF Connect for VS Code and nRF Connect for VS Code Extension extensions  
2). Clone this repo and note the nRF Connect SDK version in the west.yml manifest and delete it   
3). Open VSCode in this repo then click on "Manage west workspace" in the extension tab, and select this repo as the workspace.  
4). If you don't delete the west.yml, it thinks there's already a workspace when there isn't.  
5). Make sure you can see a folder structure that contains application, external, .west, .vscode  
6). If you can't, make sure that updating the west workspace succeeds (it pulls in external libraries)  
7). After this all succeeds, create a build configuration with the nrf52840dk as the board. All the other fields should match the files inside of this repo (aka change the overlay to the overlay this repo uses, etc)  
8). It should build now  