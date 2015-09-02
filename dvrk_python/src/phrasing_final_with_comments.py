from robot import *
import math



def dictionary(robotName):
    r=robot(robotName)
    dict = {}
    #this dictionary keeps the list of actions used to make each letter (more info below)
    dict['a'] = [('d'),('r',80,60),('r',280,30),('r',180,10),('r',0,10),('r',280,30)]
    dict['b'] = [('d'),('r',90,60),('r',350,30),('r',270,20),('r',190,30),('r',350,40),('r',270,23),('r',180,40)] 
    dict['c'] = [('r',0,35),('d'),('r',180,30),('r',108.43,15.8114),('r',90,30),('r',71.57,15.8114),('r',0,30)]
    dict['d'] = [('d'),('r',90,60),('r',340,30),('r',270,40),('r',200,30)]
    dict['e'] = [('r',0,40),('d'),('r',180,40),('r',90,30),('r',0,30),('r',180,30),('r',90,30),('r',0,40)]
    dict['f'] = [('d'),('r',90,30),('r',0,30),('r',180,30),('r',90,30),('r',0,40)]
    dict['g'] = [('r',0,40),('r',90,35),('r',180,12),('d'),('r',0,12),('r',270,35),('r',180,40),('r',90,60),('r',0,40)]
    dict['h'] = [('d'),('r',90,60),('r',270,30),('r',0,40),('r',90,30),('r',270,60)]
    dict['i'] = [('d'),('r',0,40),('r',180,20),('r',90,60),('r',180,20),('r',0,40)]
    dict['j'] = [('r',90,16),('d'),('r',306.86,15),('r',0,12),('r',36.86,15),('r',90,40),('r',180,12),('r',0,24)]
    dict['k'] = [('d'),('r',90,60),('r',270,30),('r',45,40),('r',225,40),('r',315,40)]
    dict['l'] = [('d'),('r',90,60),('r',270,60),('r',0,40)]
    dict['m'] = [('d'),('r',90,60),('r',288.44,63),('r',71.56,63),('r',270,60)]
    dict['n'] = [('d'),('r',90,60),('r',303.69,72),('r',90,60)]
    dict['o'] = [('r',0,40),('d'),('r',180,40),('r',90,60),('r',0,40),('r',270,60)]
    dict['p'] = [('d'),('r',90,60),('r',0,40),('r',270,30),('r',180,40)]
    dict['q'] = [('r',0,40),('d'),('r',180,40),('r',90,60),('r',0,40),('r',270,60),('r',135,12),('r',315,24)]
    dict['r'] = [('d'),('r',90,60),('r',0,40),('r',270,30),('r',180,40),('r',315,45)]
    dict['s'] = [('d'),('r',0,40),('r',90,30),('r',180,40),('r',90,30),('r',0,40)]
    dict['t'] = [('r',0,20),('d'),('r',90,60),('r',180,20),('r',0,40)]
    dict['u'] = [('r',90,60),('d'),('r',270,60),('r',0,40),('r',90,60)]
    dict['v'] = [('r',0,20),('d'),('r',108.44,63),('r',288.44,63),('r',71.56,63)]
    dict['w'] = [('r',90,60),('d'),('r',270,60),('r',0,20),('r',90,40),('r',270,40),('r',0,20),('r',90,60)]
    dict['x'] = [('d'),('r',56.31,72),('r',236.31,36),('r',123.69,36),('r',303.69,72)]
    dict['y'] = [('r',0,20),('d'),('r',90,30),('r',56.31,36),('r',236.31,36),('r',123.69,36)]
    dict['z'] = [('r',56.31,72),('r',180,40),('d'),('r',0,40),('r',236.31,72),('r',0,40)]
    dict['?'] = [('r',0,20),('d'),('u'),('r',90,10),('d'),('r',90,25),('r',0,20),('r',90,25),('r',180,40)]


 

    start_x = -0.1 #sets the starting position for the x axis at -.1
    start_y = 0.04 #sets the starting position for the y axis at .04
    r.move_cartesian([start_x,start_y,-0.12]) #moves robot to start position
    rot_1 = Rotation.Identity() #sets to standard rotation
    r.move_cartesian_rotation(rot_1) #^^^
    letter_number = 0 #used to keep the letters spaced out evenly
    word_number = 0   #used to keep the words spaced out evenly
    line_number = 0   #used to keep the lines spaced out evenly
 
    cycle_number = 0 #counts the position in the list of actions to create a letter
    phrase_cycle = 0 #counts the position in the list of letters
    word_count = 0   #counts the position in the list of words
    line_count = 0   #counts the position in the list of lines

    phrase = raw_input('enter the words you would like writen: ').lower() #asks the user what they would like writen and automatically makes sure its all lower case so the code understands it 
    phrase = phrase.split()  #makes the sentence a list of words

    list_of_words = [] #formats user input as a list of words where each word is a list
    for i in range(0, len(phrase)):
        n = list(phrase[i])
        list_of_words.append(n)


    counter_characters = 0 #checks number of characters per line
    draw_list = [] #list of words per line
    word = [] #list of characters in word per line
    for i in range (0,len(list_of_words)): #formats the words into lines of text
       
        word_length = len(list_of_words[i])
        if counter_characters + word_length <= 11: #checks to see if the next word will make the line longer then 11 characters
            counter_characters += word_length + 1 #if it isn't longer, it will add that word to the line
            word.append(list_of_words[i])
            print list_of_words[i]
            if i == len(list_of_words)-1:
                   draw_list.append(word)      
        else:
            draw_list.append(word) #if line is longer then 11 characters, it will add the whole line of words, minus the new, to draw_list
            word = [] #then it will set the next line to have no words in it, and add the new word to that next line
            counter_characters = len(list_of_words[i])+1
            word.append(list_of_words[i])
            if i == len(list_of_words)-1:
                   draw_list.append(list_of_words[i])  
        

    print draw_list #prints the completly formated list, where the first list determines the line, the second determines the word, and the third determines the letter

    for line_count in range (0,3): #keeps track of what line the robot is on

        for word_count in range (0,len(draw_list[line_count])): # keeps track of what word the robot is on

            length_of_word = len(draw_list[line_count][word_count]) #finds the number of letters in a word for the next loop

            for phrase_cycle in range (0, length_of_word): #keeps track of what letter the robot is on
                length_of_letter_list = len(dict[draw_list[line_count][word_count][phrase_cycle]])  #finds the length of the list of actions in creating a given letter for the next loop
               
                print draw_list[line_count][word_count][phrase_cycle] #prints out the current letter being written

                r.delta_move_cartesian([0.0,0.0,0.01])
                r.move_cartesian_translation([start_x+letter_number+word_number,start_y+line_number,-0.11]) #before writing another letter, the robot moves to the next letter, word, or line's starting position, the pen itself starts in the up position

                for cycle_number in range (0, length_of_letter_list): #this loop goes through the list of actions for making a letter, there are three actions, move the pen in the xy direction, move the pen up, and move the pen down; these are represented by r, u and d respectively
                    if dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][0] == 'r':        # if r is the first thing detected in the next action's list, then the robot will move in the xy direction based on the angle and distance whihc are specified after the r like so: ('r',angle,distance) NOTE: the angle is based on a unit circle with 0 degrees being straight to the right, 90 being up, 180 being left, and 270 being down.
                        angle = math.radians(dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][1])
                        length = (dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][2])/2500.0

                   
                        x_position = math.cos(angle)*length #these functions convert the polar coordinate into cartesian x and y distances
                        y_position = math.sin(angle)*length

                        r.delta_move_cartesian([x_position,y_position,0.0]) #the x and y distances are then used to move the robot
                      

                    elif dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number] == 'u':       # if u is the first thing detected in the next action's list, then the robot's pen will move up
                        r.delta_move_cartesian([0.0,0.0,0.01])
                    elif dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number] == 'd':        # if d is the first thing detected in the next action's list, then the robot's pen will move down
                        r.delta_move_cartesian([0.0,0.0,-0.01])


                letter_number += .02 #makes the pen move to the next letter's spot after writing a letter
            word_number += .02 #adds a space after each word
        letter_number = 0  #resets letter_number so no letters are skipped
        word_number = 0    #resets word_number so there are no extra spaces
        line_number -= .04 #tells the pen to start one line lower; we move in the negative y direction for the next line, seeing as we start at the top and move down



if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dictionary(sys.argv[1])
