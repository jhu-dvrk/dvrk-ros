from robot import *
import math



def dictionary(robotName):
    r=robot(robotName)
    dict = {}
    dict[' '] = [('u'),('d')]
    dict['test2'] = [('r',90,60),('r',180,60),('r',270,60),('r',0,60)]

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
    start_y = 0.04
    r.move_cartesian([start_x,start_y,-0.12])
    rot_1 = Rotation.Identity() #sets to standard rotation
    #rot_1 = Rotation(0.47,0.87,-0.17,0.80,-0.50,-0.34,-0.38,0.02,-0.92)
    r.move_cartesian_rotation(rot_1)
    letter_number = 0 #used to keep the letters spaced out evenly
    word_number = 0   #used to keep the words spaced out evenly
    line_number = 0   #used to keep the lines spaced out evenly
 
    
    cycle_number = 0 #counts the position in the list of actions to create a letter
    phrase_cycle = 0 #counts the position in the list of letters
    word_count = 0   #counts the position in the list of words
    line_count = 0   #counts the position in the list of lines

    phrase = raw_input('enter the words you would like writen: ').lower()
    phrase = phrase.split()

    list_of_words = [] #formats user input as a list of words where each word is a list
    for i in range(0, len(phrase)):
        n = list(phrase[i])
        list_of_words.append(n)

    print list_of_words

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
        

    print draw_list

    for line_count in range (0,3):


        for word_count in range (0,len(draw_list[line_count])):

            length_of_word = len(draw_list[line_count][word_count])

            for phrase_cycle in range (0, length_of_word):
                length_of_letter_list = len(dict[draw_list[line_count][word_count][phrase_cycle]])  
               
                print draw_list[line_count][word_count][phrase_cycle]
                print 'wn',word_number

                r.delta_move_cartesian([0.0,0.0,0.01])
                r.move_cartesian_translation([start_x+letter_number+word_number,start_y+line_number,-0.11]) #before writing another letter, the robot moves to the next letter, word, or line's starting position

                for cycle_number in range (0, length_of_letter_list): 
                    if dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][0] == 'r':        # ('r',angle,distance)
                        angle = math.radians(dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][1])
                        length = (dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number][2])/2500.0

                   
                        x_position = math.cos(angle)*length
                        y_position = math.sin(angle)*length

                        r.delta_move_cartesian([x_position,y_position,0.0])
                      

                    elif dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number] == 'u':         #u = pen up
                        r.delta_move_cartesian([0.0,0.0,0.01])
                    elif dict[draw_list[line_count][word_count][phrase_cycle]][cycle_number] == 'd':         #d = pen down
                        r.delta_move_cartesian([0.0,0.0,-0.01])


                letter_number += .02
            word_number += .02
        letter_number = 0
        word_number = 0
        line_number -= .04



if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dictionary(sys.argv[1])
