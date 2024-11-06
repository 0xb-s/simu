use eframe::{egui, App, Frame, NativeOptions};
use egui::{
    plot::{Line, Plot, PlotPoints},
    FontId, Pos2,
};
use petgraph::{
    graph::{DiGraph, NodeIndex},
    visit::Topo,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;


#[derive(Debug, Clone, Serialize, Deserialize)]
struct Position {
    x: f32,
    y: f32,
}

impl From<egui::Pos2> for Position {
    fn from(pos: egui::Pos2) -> Self {
        Position { x: pos.x, y: pos.y }
    }
}

impl Into<egui::Pos2> for Position {
    fn into(self) -> egui::Pos2 {
        egui::pos2(self.x, self.y)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
enum ComponentType {
    Step,
    TransferFunction,
    Scope,
    Delay(usize),       
    Difference,         
    DiscreteDerivative, 
    DiscreteIntegrator,
    PIDController {
    
        kp: f32,
        ki: f32,
        kd: f32,
    },
    Memory, 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Component {
    id: usize,
    component_type: ComponentType,
    position: Position,
    is_dragging: bool,
}

struct SimulatorApp {
    components: HashMap<usize, Component>,
    connections: DiGraph<usize, f32>,
    next_id: usize,
    selected_component: Option<usize>,
    simulation_data: Vec<f32>, 
}

impl SimulatorApp {
    fn new() -> Self {
        SimulatorApp {
            components: HashMap::new(),
            connections: DiGraph::new(),
            next_id: 0,
            selected_component: None,
            simulation_data: vec![],
        }
    }

    fn add_component(&mut self, component_type: ComponentType, position: egui::Pos2) -> NodeIndex {
        let id = self.next_id;
        self.next_id += 1;
        let component = Component {
            id,
            component_type,
            position: position.into(),
            is_dragging: false,
        };
        self.components.insert(id, component);
        self.connections.add_node(id)
    }

    fn connect_components(&mut self, from: usize, to: usize) {
        if let (Some(from_idx), Some(to_idx)) = (
            self.connections.node_indices().find(|&n| n.index() == from),
            self.connections.node_indices().find(|&n| n.index() == to),
        ) {
            self.connections.add_edge(from_idx, to_idx, 1.0);
        }
    }


    fn simulate(&mut self) {
      
        self.simulation_data.clear();

        let time_step = 0.1;
        let steps = 100;
        let mut component_outputs = HashMap::new();

        for step in 0..steps {
            println!("Simulation step {}", step);

            let mut topo = Topo::new(&self.connections);
            while let Some(node_idx) = topo.next(&self.connections) {
                let component_id = self.connections[node_idx];

                if let Some(component) = self.components.get(&component_id) {
                    let output = match &component.component_type {
                        ComponentType::Step => 1.0,
                        ComponentType::TransferFunction => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            let prev_output = *component_outputs.get(&component_id).unwrap_or(&0.0);
                            let alpha = 0.1;
                            prev_output + alpha * (input_value - prev_output)
                        }
                        ComponentType::Scope => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            self.simulation_data.push(input_value);
                            continue;
                        }
                        ComponentType::Delay(delay_steps) => {
                            todo!()
                        }
                        ComponentType::Difference => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            let prev_value =
                                *component_outputs.get(&component_id).unwrap_or(&input_value);
                            input_value - prev_value
                        }
                        ComponentType::DiscreteDerivative => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            let prev_value =
                                *component_outputs.get(&component_id).unwrap_or(&input_value);
                            (input_value - prev_value) / time_step
                        }
                        ComponentType::DiscreteIntegrator => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            let prev_value = *component_outputs.get(&component_id).unwrap_or(&0.0);
                            prev_value + input_value * time_step
                        }
                        ComponentType::PIDController { kp, ki, kd } => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            let prev_error =
                                *component_outputs.get(&(component_id + 1)).unwrap_or(&0.0);
                            let prev_integral =
                                *component_outputs.get(&(component_id + 2)).unwrap_or(&0.0);
                            let error = 1.0 - input_value; 
                            let integral = prev_integral + error * time_step;
                            let derivative = (error - prev_error) / time_step;
                            *kp * error + *ki * integral + *kd * derivative
                        }
                        ComponentType::Memory => {
                            let input_value =
                                self.get_input_value(component_id, &component_outputs);
                            *component_outputs.get(&component_id).unwrap_or(&input_value)
                        }
                    };

                    component_outputs.insert(component_id, output);
                    println!(
                        "Component ID {} ({:?}) output: {}",
                        component_id, component.component_type, output
                    );
                }
            }
        }
    }
    fn add_delay(&mut self, delay_steps: usize, position: egui::Pos2) {
        let component_type = ComponentType::Delay(delay_steps);
        self.add_component(component_type, position);
    }

    fn add_difference(&mut self, position: egui::Pos2) {
        self.add_component(ComponentType::Difference, position);
    }

    fn add_discrete_derivative(&mut self, position: egui::Pos2) {
        self.add_component(ComponentType::DiscreteDerivative, position);
    }

    fn add_discrete_integrator(&mut self, position: egui::Pos2) {
        self.add_component(ComponentType::DiscreteIntegrator, position);
    }

    fn add_pid_controller(&mut self, kp: f32, ki: f32, kd: f32, position: egui::Pos2) {
        let component_type = ComponentType::PIDController { kp, ki, kd };
        self.add_component(component_type, position);
    }

    fn add_memory(&mut self, position: egui::Pos2) {
        self.add_component(ComponentType::Memory, position);
    }

    fn get_input_value(&self, component_id: usize, component_outputs: &HashMap<usize, f32>) -> f32 {
        let mut input_sum = 0.0;

        
        if let Some(node_idx) = self
            .connections
            .node_indices()
            .find(|n| self.connections[*n] == component_id)
        {
            for neighbor in self
                .connections
                .neighbors_directed(node_idx, petgraph::Incoming)
            {
                if let Some(&output_value) = component_outputs.get(&self.connections[neighbor]) {
                    input_sum += output_value;
                }
            }
        }

        println!(
            "Component ID {} received input value: {}",
            component_id, input_sum
        ); 

        input_sum
    }
}

impl App for SimulatorApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
    
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Add Step").clicked() {
                    self.add_component(ComponentType::Step, Pos2::new(50.0, 100.0));
                }
                if ui.button("Add Transfer Function").clicked() {
                    self.add_component(ComponentType::TransferFunction, Pos2::new(150.0, 100.0));
                }
                if ui.button("Add Scope").clicked() {
                    self.add_component(ComponentType::Scope, Pos2::new(250.0, 100.0));
                }
                if ui.button("Run Simulation").clicked() {
                    self.simulate();
                }
            });
        });

      
        egui::SidePanel::left("side_panel").show(ctx, |ui| {
            ui.heading("Simulation Output");

        
            let plot_points: PlotPoints = PlotPoints::from_iter(
                self.simulation_data
                    .iter()
                    .enumerate()
                    .map(|(i, &value)| [i as f64 * 0.1, value as f64]), 
            );

         
            let line = Line::new(plot_points).name("Simulation Result");

            Plot::new("Scope Plot")
                .view_aspect(2.0) 
                .show(ui, |plot_ui| {
                    plot_ui.line(line);
                });
        });

     
        egui::CentralPanel::default().show(ctx, |ui| {
            let painter = ui.painter();
            let mut connection_to_create = None; 

        
            for edge in self.connections.edge_indices() {
                let (from, to) = self.connections.edge_endpoints(edge).unwrap();
                let from_pos: Pos2 = self.components[&self.connections[from]]
                    .position
                    .clone()
                    .into();
                let to_pos: Pos2 = self.components[&self.connections[to]]
                    .position
                    .clone()
                    .into();
                painter.line_segment([from_pos, to_pos], (1.0, egui::Color32::LIGHT_GRAY));
            }

        
            for (id, component) in self.components.iter_mut() {
                let pos: Pos2 = component.position.clone().into();
                let rect = egui::Rect::from_center_size(pos, egui::vec2(80.0, 40.0));

                let color = match component.component_type {
                    ComponentType::Step => egui::Color32::LIGHT_BLUE,
                    ComponentType::TransferFunction => egui::Color32::LIGHT_YELLOW,
                    ComponentType::Scope => egui::Color32::LIGHT_GREEN,
                    _ => todo!(),
                };

              
                painter.rect_filled(rect, 5.0, color);
                let label = match component.component_type {
                    ComponentType::Step => "Step",
                    ComponentType::TransferFunction => "1 / (s + 1)",
                    ComponentType::Scope => "Scope",
                    _ => todo!(),
                };
                painter.text(
                    pos,
                    egui::Align2::CENTER_CENTER,
                    label,
                    FontId::default(),
                    egui::Color32::BLACK,
                );

                // Handle dragging
                if ui.rect_contains_pointer(rect) && ui.input().pointer.any_pressed() {
                    component.is_dragging = true;
                }
                if ui.input().pointer.any_released() {
                    component.is_dragging = false;
                }
                if component.is_dragging {
                    if let Some(mouse_pos) = ui.input().pointer.hover_pos() {
                        component.position = mouse_pos.into();
                    }
                }

        
                if ui.rect_contains_pointer(rect) && ui.input().pointer.any_click() {
                    if let Some(start_id) = self.selected_component {
                        if start_id != *id {
                            connection_to_create = Some((start_id, *id));
                        }
                        self.selected_component = None;
                    } else {
                        self.selected_component = Some(*id);
                    }
                }
            }

         
            if let Some((start_id, end_id)) = connection_to_create {
                self.connect_components(start_id, end_id);
            }
        });
    }
}

fn main() {
    let options = NativeOptions::default();
    eframe::run_native(
        " Simulator",
        options,
        Box::new(|_cc| Box::new(SimulatorApp::new())),
    );
}
