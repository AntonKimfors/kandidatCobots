'''
Function to generate a product order for testing purposes in the machine assembly
'''

import random


def generate_product_order(numberOfProducts: int, productList: list,
                           order=random):
    ''' Generates a product order of specified size, products and order.

    Keyword arguments: \n

    numberOfProducts -- How many products are to be made

    productList -- List of product names to be made

    order --  What order you want the products to be made in \t
            random(default) or alternating
    '''
    generated_product_order = []

    for i in range(numberOfProducts):
        if order == random:
            nextProduct = productList[random.randrange(len(productList))]
        else:
            nextProduct = productList[i % 2]

        generated_product_order.append(nextProduct)

    return generated_product_order

