from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from math import sin, cos, sqrt, atan2, radians
from decimal import Decimal
# [END import]

latitudes = ["-33,456352","-33,453589","-33,453296","-33,458926","-33,456123","-33,506002","-33,536039","-33,541844","-33,554147","-33,56154","-33,570992","-33,619779","-33,618463","-33,609205","-33,605217","-33,602055","-33,597229","-33,592432","-33,601232","-33,5968699","-33,610844","-33,5987165","-33,594355","-33,585834","-33,586994","-33,572861","-33,548271","-33,55526","-33,564902","-33,571603","-33,573354","-33,577954","-33,581824","-33,582426","-33,456352","-33,41239","-33,366926","-33,366359","-33,377516","-33,378708","-33,391961","-33,401329","-33,406213","-33,395366","-33,418567","-33,389634","-33,396111","-33,387271","-33,393058","-33,396931","-33,405485","-33,40671","-33,456352","-33,467075","-33,4587572","-33,453431","-33,438402","-33,437689","-33,4366689","-33,445228","-33,444747","-33,449218","-33,449341","-33,452961","-33,4555715","-33,45867","-33,460382","-33,459582","-33,455266","-33,483443","-33,479512","-33,497366","-33,495371","-33,495371","-33,509625","-33,517028","-33,519713","-33,515492","-33,513094","-33,504348","-33,523753","-33,456352","-33,444009","-33,441433","-33,438472","-33,436503","-33,433264","-33,431813","-33,429557","-33,437554","-33,443471","-33,4515113","-33,451511","-33,453626","-33,4503504","-33,4466359","-33,441945","-33,439786","-33,435298","-33,433948","-33,4571106","-33,492034","-33,562034","-33,456352","-33,445295","-33,452334","-33,452371","-33,450922","-33,4541346","-33,460616","-33,4693177","-33,4668828","-33,470817","-33,47003","-33,482993","-33,486535","-33,486081","-33,479333","-33,457773","-33,452982","-33,46304","-33,464434","-33,487501","-33,5016852","-33,4973","-33,510972","-33,517947","-33,511008","-33,515573","-33,51393","-33,527295","-33,524934","-33,533437","-33,53955","-33,537248","-33,536916","-33,456352","-33,492878","-33,495269","-33,508779","-33,496563","-33,505852","-33,526967","-33,526203","-33,53279","-33,550643","-33,557444","-33,571253","-33,601507","-33,617635","-33,637619","-33,643639","-33,734532","-33,6159802","-33,603564","-33,593162","-33,589492","-33,601894","-33,606221","-33,456352","-33,450569","-33,473269","-33,464599","-33,457129","-33,443354","-33,418779","-33,38718","-33,383985","-33,3541754","-33,350321","-33,370304","-33,387285","-33,395181","-33,398545","-33,403356","-33,419049","-33,456352","-33,412833","-33,418979","-33,423507","-33,414066","-33,392631","-33,401067","-33,403921","-33,397164","-33,392146","-33,3853462","-33,400403","-33,409694","-33,410795","-33,41202","-33,411678","-33,425214","-33,425056","-33,428668","-33,430044","-33,430572","-33,437665","-33,424888","-33,456352","-33,483856","-33,510587","-33,518292","-33,515747","-33,531671","-33,53556","-33,523405","-33,521219","-33,514753","-33,50495","-33,6045833","-33,561585","-33,456352","-33,211163","-33,183339","-33,204545","-33,249391","-33,317802","-33,323536","-33,333418","-33,339679","-33,349991","-33,35924","-33,358527","-33,359656","-33,345114"]
longitudes = ["-70,665234","-70,660114","-70,657073","-70,653753","-70,655999","-70,622069","-70,595074","-70,594338","-70,627785","-70,619052","-70,608059","-70,623078","-70,583783","-70,58287","-70,579278","-70,571975","-70,582153","-70,569676","-70,547368","-70,5412722","-70,531028","-70,5197362","-70,547294","-70,554367","-70,559832","-70,543993","-70,557259","-70,569284","-70,564948","-70,561168","-70,561495","-70,572824","-70,587569","-70,585602","-70,665234","-70,669332","-70,667319","-70,66806","-70,664792","-70,657659","-70,648202","-70,648079","-70,619499","-70,629013","-70,611224","-70,593427","-70,57986","-70,569941","-70,566728","-70,568752","-70,592626","-70,596223","-70,665234","-70,691111","-70,6921346","-70,685506","-70,70774","-70,688526","-70,6726577","-70,671918","-70,661152","-70,6557662","-70,655092","-70,634343","-70,6232142","-70,622888","-70,619997","-70,62227","-70,629518","-70,650142","-70,653428","-70,66125","-70,657849","-70,657849","-70,653469","-70,656219","-70,656742","-70,663711","-70,664293","-70,672088","-70,66891","-70,665234","-70,646472","-70,639194","-70,641516","-70,657145","-70,653307","-70,650586","-70,627654","-70,628925","-70,616048","-70,6132677","-70,613268","-70,607308","-70,6066717","-70,596165","-70,595645","-70,600028","-70,614322","-70,606032","-70,6077837","-70,612315","-70,634816","-70,665234","-70,639487","-70,628729","-70,628763","-70,630207","-70,6221906","-70,600807","-70,5918086","-70,5861289","-70,571586","-70,569998","-70,565693","-70,541182","-70,539131","-70,544246","-70,551113","-70,566085","-70,576443","-70,575529","-70,583517","-70,582612","-70,554583","-70,554671","-70,530065","-70,520471","-70,57326","-70,592627","-70,585223","-70,575914","-70,575532","-70,56604","-70,565734","-70,571225","-70,665234","-70,735471","-70,734346","-70,732638","-70,724954","-70,709266","-70,652422","-70,640708","-70,650971","-70,660794","-70,671481","-70,701553","-70,703968","-70,708766","-70,70935","-70,697735","-70,742808","-70,6846941","-70,694099","-70,695949","-70,683401","-70,678996","-70,680105","-70,665234","-70,720654","-70,746123","-70,746275","-70,758912","-70,740451","-70,758961","-70,817727","-70,732437","-70,7155659","-70,680232","-70,691338","-70,686432","-70,699543","-70,699333","-70,710871","-70,698806","-70,665234","-70,578124","-70,563579","-70,550647","-70,548636","-70,509744","-70,524401","-70,561304","-70,565467","-70,557376","-70,5539633","-70,572903","-70,574367","-70,576738","-70,579619","-70,58687","-70,587692","-70,591425","-70,595514","-70,596276","-70,594237","-70,577579","-70,577372","-70,665234","-70,765974","-70,757261","-70,754347","-70,770092","-70,782836","-70,785954","-70,792924","-70,796482","-70,78731","-70,778075","-70,9051066","-70,633515","-70,665234","-70,725419","-70,677059","-70,675965","-70,625923","-70,564955","-70,515216","-70,511061","-70,494709","-70,511701","-70,505146","-70,502096","-70,503017","-70,476498"]

latfin = ["-33.582426","-33.40671","-33.523753","-33.562034","-33.536916","-33.606221","-33.419049","-33.424888","-33.561585","-33.345114"]
indexes = []




u = 0

while u < len(latitudes):
    latitudes[u] = latitudes[u].replace(",",'.')
    longitudes[u] = longitudes[u].replace(",",'.')
    u += 1


j = 0

while j < len(latfin):
    ind = latitudes.index(latfin[j])
    indexes.append(ind)
    j += 1


matrixD = []

i = 0

while i < len(latitudes):

    e = 0
    temp = []
    while e < len(latitudes):

        R = 6373.0

        lat1 = radians(Decimal(latitudes[i]))
        lon1 = radians(Decimal(longitudes[i]))
        lat2 = radians(Decimal(latitudes[e]))
        lon2 = radians(Decimal(longitudes[e]))

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c
        temp.append(distance*3.5 + 6)
        e += 1
    matrixD.append(temp)
    i += 1



# [START data_model]
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = matrixD
    data['num_vehicles'] = 10
    # [START starts_ends]
    data['ends'] = indexes
    data['starts'] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # [END starts_ends]
    return data
    # [END data_model]


# [START solution_printer]
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    resultados = []
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        tempp = []
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            tempp.append(format(manager.IndexToNode(index)))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        resultados.append(tempp)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}km'.format(max_route_distance))
    print(resultados)
    print(indexes)
    # [END solution_printer]


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    # [START data]
    data = create_data_model()
    # [END data]

    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['starts'],
                                           data['ends'])
    # [END index_manager]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)

    # [END routing_model]

    # Create and register a transit callback.
    # [START transit_callback]
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # [END transit_callback]

    # Define cost of each arc.
    # [START arc_cost]
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # [END arc_cost]

    # Add Distance constraint.
    # [START distance_constraint]
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        480,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    # [END distance_constraint]

    # Setting first solution heuristic.
    # [START parameters]
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # [END parameters]

    # Solve the problem.
    # [START solve]
    solution = routing.SolveWithParameters(search_parameters)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    if solution:
        print_solution(data, manager, routing, solution)
    # [END print_solution]


if __name__ == '__main__':
    main()
    # [END program]