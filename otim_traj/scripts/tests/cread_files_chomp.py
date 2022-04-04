#! /usr/bin/env python


# open the sample file used
file = open('ompl.03-03-2022_12_34_43_-03.txt')
  
# read the content of the file opened
content = file.readlines()
#stomp
print("Planning time:")
print(content[7], content[10], content[16], content[28], content[37], content[40])

print("Trajectory Lenght:")
print(content[8], content[11], content[17], content[29], content[38], content[41])  

  



# chomp
# print("SUCCESS:")
# print(content[6], content[9], content[15], content[33], content[42], content[45])

# print("Planning time:")
# print(content[7], content[10], content[16], content[34], content[43], content[46])

# print("Trajectory Lenght:")
# print(content[8], content[11], content[17], content[35], content[44], content[47])