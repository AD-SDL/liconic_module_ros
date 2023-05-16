import json
import datetime


class Resource():
    def __init__(self):
        #TODO: import path from client
        self.resources = json.load(open('/home/rpl/liconic_temp/resources/liconic_resources.json'))

    def add_plate(self, plate_id, stack = None, slot = None): #TODO: add parameter for identifying plate type if we have multiple types of stacks in liconic
        '''
        updates the liconic resource file when a new plate is placed into the liconic
        '''
        if stack != None and slot != None:
            stack, slot = self.convert_stack_and_slot_key(stack, slot)
            if self.resources[stack][slot]["occupied"] == True:
                print("ERROR: liconic position: " + str(stack) + str(slot) + " already occupied")
            else:
                self.resources[stack][slot]["occupied"] = True
                self.resources[stack][slot]["plate_id"] = plate_id
                self.resources[stack][slot]["time_added"] = datetime.datetime.now()
                self.update_resource_file()
        else:
            stack, slot = self.get_next_free_slot_int()
            if self.resources[stack][slot]["occupied"] == True:
                print("ERROR: liconic position: " + str(stack) + str(slot) + " already occupied")
            else:
                self.resources[stack][slot]["occupied"] = True
                self.resources[stack][slot]["plate_id"] = plate_id
                self.resources[stack][slot]["time_added"] = datetime.datetime.now()
                self.update_resource_file()

    def remove_plate(self, plate_id, stack = None, slot = None):
        '''
        locates and removes the given plate from the resource file
        '''
        if stack == None and slot == None: #TODO what if user gives EITHER slot or stack
            stack, slot = self.find_plate(plate_id)

        if self.resources[stack][slot]["occupied"] == False:
            print("ERROR: liconic position: " + str(stack) + str(slot) + " has no plate")
        else:
            self.resources[stack][slot]["occupied"] = False
            self.resources[stack][slot]["plate_id"] = "NONE"
            # self.resources[stack][slot]["time_added"] = datetime.datetime.now()
            self.update_resource_file()
    
    def find_plate(self, plate_id):
        """
        returns the stack and slot a plate is located on, given the plate id
        """
        key_list = list(self.resources.keys())
        value_list = list(self.resources.values())
        for stack in range(len(key_list)):
            for value in value_list[stack]:
                if self.resources[key_list[stack]][value]['plate_id'] == plate_id:
                    return key_list[stack], value
    
    def get_next_free_slot(self):
        '''
        if no stack and shelf is passed into add_plate, return the next free location
        '''
        key_list = list(self.resources.keys())
        value_list = list(self.resources.values())
        for stack in range(len(key_list)):
            for value in value_list[stack]:
                if self.resources[key_list[stack]][value]['occupied'] == False:
                    return key_list[stack], value
    
    def get_next_free_slot_int(self):
        stack, slot = self.get_next_free_slot()
        stack_int = int(stack[-1])
        slot_int = int(slot[-1])
        return stack_int, slot_int

    def convert_stack_and_slot_int(self, stack, slot):
        '''
        converts stack and slot dict keys to ints
        '''
        stack_int = int(stack[-1])
        slot_int = int(slot[-1])
        return stack_int, slot_int

    def convert_stack_and_slot_key(self, stack, slot):
        '''
        converts stack and slot ints into dict keys
        '''
        new_stack = "Stack"+str(stack)
        new_slot = "Slot"+str(slot)
        return new_stack, new_slot
    
    def is_location_occupied(self, stack, slot):
        '''
        given a stack and slot, determine if the location is occupied
        '''
        return self.resources[stack][slot]["occupied"]
    
    def check_existing_id(self, plate_id):
        '''
        will check the given plate id against the resource file to determine if there is a plate with the same id in the liconic
        '''
        key_list = list(self.resources.keys())
        value_list = list(self.resources.values())
        for stack in range(len(key_list)):
            for value in value_list[stack]:
                if self.resources[key_list[stack]][value]['plate_id'] == plate_id:
                    return -1
        
        return 0

        
        
    def update_resource_file(self):
        '''
        updates the external resource file to match self.resources
        '''
        with open('liconic_resources.json', 'w') as f:
            json.dump(self.resources, f)
    
if __name__ == "__main__":
    test = Resource()
    # print(test.get_next_free_slot())
    # print(test.get_next_free_slot_int())
    # print(test.check_existing_id('abe'))
    # print(test.convert_stack_and_slot(3, 4))