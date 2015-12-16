def outlier_test():
    import math

    alist = [1,2,3,4,5,6,7,9001,8,0]   #list(int(raw_input("type a list \n")))
    print alist
    asortedlist = sorted(alist)
    asortedlist2 = asortedlist
    print asortedlist
    if len(asortedlist) % 2 == 0:
        median = float(asortedlist[(len(asortedlist)/2)-1] + asortedlist[(len(asortedlist)/2)])/2
        Q1list = asortedlist[:-((len(asortedlist))/2)]
        Q3list = asortedlist[-((len(asortedlist))/2):]
    elif len(asortedlist) % 2 == 1:
        median = asortedlist[int((len(asortedlist)/2)+.5)]
        Q1list = asortedlist[:-int(((len(asortedlist))/2)+1.5)]
        Q3list = asortedlist[-int(((len(asortedlist))/2)+.5):]
    if len(Q1list) % 2 == 0:
        Q1 = float(Q1list[(len(Q1list)/2)-1] + Q1list[(len(Q1list)/2)])/2
    elif len(Q1list) % 2 == 1:
        Q1 = Q1list[int((len(Q1list)/2)+.5)]
    if len(Q3list) % 2 == 0:
        Q3 = float(Q3list[(len(Q3list)/2)-1] + Q3list[(len(Q3list)/2)])/2
    elif len(Q3list) % 2 == 1:
        Q3 = Q3list[int((len(Q3list)/2)+.5)]
    IQR = Q3 - Q1
    for i in asortedlist:
        if i > ((1.5 * IQR) + median) or i < ((-1.5 * IQR) + median):
            asortedlist2.remove(i)
    print asortedlist2
outlier_test()
